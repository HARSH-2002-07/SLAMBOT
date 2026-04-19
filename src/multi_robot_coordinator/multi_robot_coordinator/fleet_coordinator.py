"""Fleet coordinator node.

Accepts navigation tasks via YAML at startup and a runtime service, then
dispatches each task to whichever robot is currently idle and nearest to the
goal. Uses a MultiThreadedExecutor so multiple robots can navigate in parallel.
"""

import math
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from multi_robot_coordinator_interfaces.msg import FleetStatus
from multi_robot_coordinator_interfaces.srv import SendGoal


STATE_IDLE = "idle"
STATE_BUSY = "busy"
STATE_ERROR = "error"


@dataclass
class Task:
    task_id: str
    x: float
    y: float
    yaw: float
    retries_left: int


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class FleetCoordinator(Node):

    def __init__(self, node_name: str = "fleet_coordinator", **kwargs):
        super().__init__(node_name, **kwargs)

        self.declare_parameter("robot_names", ["robot_1", "robot_2"])
        self.declare_parameter("task_file", "")
        self.declare_parameter("max_retries", 1)
        self.declare_parameter("dispatch_hz", 2.0)

        self.robot_names: List[str] = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.task_file: str = (
            self.get_parameter("task_file").get_parameter_value().string_value
        )
        self.max_retries: int = (
            self.get_parameter("max_retries").get_parameter_value().integer_value
        )
        dispatch_hz: float = (
            self.get_parameter("dispatch_hz").get_parameter_value().double_value
        )

        self._cb_group = ReentrantCallbackGroup()
        # RLock so _publish_status() can be called from inside already-locked sections.
        self._lock = threading.RLock()
        self._queue: List[Task] = []
        self._task_counter = 0

        self._states: Dict[str, str] = {r: STATE_IDLE for r in self.robot_names}
        self._poses: Dict[str, Optional[PoseWithCovarianceStamped]] = {
            r: None for r in self.robot_names
        }
        self._active_goal_handles: Dict[str, object] = {r: None for r in self.robot_names}

        self._action_clients: Dict[str, ActionClient] = {}
        for r in self.robot_names:
            self._action_clients[r] = ActionClient(
                self,
                NavigateToPose,
                f"/{r}/navigate_to_pose",
                callback_group=self._cb_group,
            )

        for r in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{r}/amcl_pose",
                lambda msg, rn=r: self._amcl_cb(rn, msg),
                10,
                callback_group=self._cb_group,
            )

        self._send_goal_srv = self.create_service(
            SendGoal,
            "/fleet_coordinator/send_goal",
            self._send_goal_cb,
            callback_group=self._cb_group,
        )

        self._status_pub = self.create_publisher(
            FleetStatus, "/fleet_coordinator/status", 10
        )

        self._dispatch_timer = self.create_timer(
            1.0 / max(dispatch_hz, 0.1),
            self._dispatch_tick,
            callback_group=self._cb_group,
        )
        self._status_timer = self.create_timer(
            1.0, self._publish_status, callback_group=self._cb_group
        )

        if self.task_file and os.path.isfile(self.task_file):
            self._load_tasks_from_yaml(self.task_file)
        elif self.task_file:
            self.get_logger().warn(f"task_file not found: {self.task_file}")

        self.get_logger().info(
            f"fleet_coordinator ready — robots={self.robot_names} "
            f"queue={len(self._queue)} max_retries={self.max_retries}"
        )

    def _load_tasks_from_yaml(self, path: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        tasks = data.get("tasks", [])
        with self._lock:
            for t in tasks:
                self._task_counter += 1
                self._queue.append(
                    Task(
                        task_id=f"task_{self._task_counter:04d}",
                        x=float(t["x"]),
                        y=float(t["y"]),
                        yaw=float(t.get("yaw", 0.0)),
                        retries_left=self.max_retries,
                    )
                )
        self.get_logger().info(f"loaded {len(tasks)} tasks from {path}")

    def _amcl_cb(self, robot: str, msg: PoseWithCovarianceStamped):
        self._poses[robot] = msg

    def _send_goal_cb(self, request: SendGoal.Request, response: SendGoal.Response):
        with self._lock:
            self._task_counter += 1
            task = Task(
                task_id=f"task_{self._task_counter:04d}",
                x=float(request.x),
                y=float(request.y),
                yaw=float(request.yaw),
                retries_left=self.max_retries,
            )
            self._queue.append(task)
        self.get_logger().info(
            f"queued {task.task_id} -> ({task.x:.2f}, {task.y:.2f}, yaw={task.yaw:.2f})"
        )
        response.success = True
        response.task_id = task.task_id
        return response

    def _dispatch_tick(self):
        with self._lock:
            if not self._queue:
                return
            idle_robots = [r for r in self.robot_names if self._states[r] == STATE_IDLE]
            if not idle_robots:
                return

            task = self._queue[0]
            robot = self._pick_nearest(idle_robots, task)
            if robot is None:
                # No robot has a known pose yet; wait for AMCL.
                return

            self._queue.pop(0)
            self._states[robot] = STATE_BUSY

        self._publish_status()
        self._send_goal_async(robot, task)

    def _pick_nearest(self, idle_robots: List[str], task: Task) -> Optional[str]:
        best_robot = None
        best_dist = float("inf")
        any_pose_known = False
        for r in idle_robots:
            pose_msg = self._poses[r]
            if pose_msg is None:
                continue
            any_pose_known = True
            dx = pose_msg.pose.pose.position.x - task.x
            dy = pose_msg.pose.pose.position.y - task.y
            d = math.hypot(dx, dy)
            if d < best_dist:
                best_dist = d
                best_robot = r
        if not any_pose_known:
            # Fallback: pick the first idle robot so we don't deadlock
            # if AMCL has published nothing (e.g. initial-pose not yet set).
            return idle_robots[0]
        return best_robot

    def _send_goal_async(self, robot: str, task: Task):
        client = self._action_clients[robot]
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f"{robot}: navigate_to_pose server not available; re-queueing {task.task_id}"
            )
            self._fail_or_requeue(robot, task, reason="server_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = task.x
        goal.pose.pose.position.y = task.y
        qx, qy, qz, qw = yaw_to_quaternion(task.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"{robot}: dispatching {task.task_id} -> ({task.x:.2f}, {task.y:.2f})"
        )
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, rn=robot, tk=task: self._goal_response_cb(rn, tk, fut)
        )

    def _goal_response_cb(self, robot: str, task: Task, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"{robot}: send_goal failed: {e}")
            self._fail_or_requeue(robot, task, reason="send_failed")
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"{robot}: goal {task.task_id} rejected")
            self._fail_or_requeue(robot, task, reason="rejected")
            return

        self._active_goal_handles[robot] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, rn=robot, tk=task: self._result_cb(rn, tk, fut)
        )

    def _result_cb(self, robot: str, task: Task, future):
        self._active_goal_handles[robot] = None
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"{robot}: result fetch failed: {e}")
            self._fail_or_requeue(robot, task, reason="result_exception")
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{robot}: {task.task_id} SUCCEEDED")
            with self._lock:
                self._states[robot] = STATE_IDLE
            self._publish_status()
        else:
            self.get_logger().warn(
                f"{robot}: {task.task_id} finished with status={status}"
            )
            self._fail_or_requeue(robot, task, reason=f"status_{status}")

    def _fail_or_requeue(self, robot: str, task: Task, reason: str):
        with self._lock:
            self._states[robot] = STATE_IDLE
            if task.retries_left > 0:
                task.retries_left -= 1
                self._queue.append(task)
                self.get_logger().warn(
                    f"re-queued {task.task_id} ({reason}); retries_left={task.retries_left}"
                )
            else:
                self.get_logger().error(
                    f"dropping {task.task_id} after retries ({reason})"
                )
        self._publish_status()

    def _publish_status(self):
        msg = FleetStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_names = list(self.robot_names)
        with self._lock:
            msg.robot_states = [self._states[r] for r in self.robot_names]
            msg.queue_size = len(self._queue)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
