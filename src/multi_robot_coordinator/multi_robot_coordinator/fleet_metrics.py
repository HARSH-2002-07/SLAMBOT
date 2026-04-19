"""Fleet metrics node.

Subscribes to /{robot}/odom for distance tracking, and to
/fleet_coordinator/status for task state transitions. Publishes a rolling
summary on /fleet_metrics/summary at 1 Hz.

Distance uses /odom (not /amcl_pose) because /odom is smooth and high-rate;
drift is negligible over a short demo.

Task accounting is derived from FleetStatus state transitions only:
  idle -> busy  = task dispatched
  busy -> idle  = task completed (the current coordinator re-queues failures
                 as idle, so we cannot distinguish success vs failure here)
"""

import math
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header

from multi_robot_coordinator_interfaces.msg import FleetMetrics, FleetStatus


class FleetMetricsNode(Node):

    def __init__(self):
        super().__init__("fleet_metrics")

        self.declare_parameter("robot_names", ["robot_1", "robot_2"])
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("status_topic", "/fleet_coordinator/status")

        self.robot_names: List[str] = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        publish_hz: float = (
            self.get_parameter("publish_hz").get_parameter_value().double_value
        )
        status_topic: str = (
            self.get_parameter("status_topic").get_parameter_value().string_value
        )

        self._lock = threading.Lock()
        self._last_xy: Dict[str, Optional[Tuple[float, float]]] = {
            r: None for r in self.robot_names
        }
        self._distance_m: Dict[str, float] = {r: 0.0 for r in self.robot_names}
        self._tasks_completed: Dict[str, int] = {r: 0 for r in self.robot_names}
        self._total_dispatched = 0

        # Per-robot busy-phase timing: start-time stamps in seconds.
        self._busy_since: Dict[str, Optional[float]] = {r: None for r in self.robot_names}
        self._completed_durations: List[float] = []

        # Last observed state per robot (for edge detection).
        self._last_state: Dict[str, str] = {r: "idle" for r in self.robot_names}

        for r in self.robot_names:
            self.create_subscription(
                Odometry,
                f"/{r}/odom",
                lambda msg, rn=r: self._odom_cb(rn, msg),
                20,
            )

        self.create_subscription(
            FleetStatus,
            status_topic,
            self._status_cb,
            10,
        )

        self._pub = self.create_publisher(FleetMetrics, "/fleet_metrics/summary", 10)
        self._timer = self.create_timer(1.0 / max(publish_hz, 0.1), self._publish_summary)

        self.get_logger().info(
            f"fleet_metrics ready — robots={self.robot_names} "
            f"publish_hz={publish_hz} status_topic={status_topic}"
        )

    def _odom_cb(self, robot: str, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            prev = self._last_xy[robot]
            if prev is not None:
                dx = x - prev[0]
                dy = y - prev[1]
                self._distance_m[robot] += math.hypot(dx, dy)
            self._last_xy[robot] = (x, y)

    def _status_cb(self, msg: FleetStatus):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            for name, state in zip(msg.robot_names, msg.robot_states):
                if name not in self._last_state:
                    continue
                prev = self._last_state[name]
                if prev == state:
                    continue
                if prev == "idle" and state == "busy":
                    self._busy_since[name] = now
                    self._total_dispatched += 1
                elif prev == "busy" and state == "idle":
                    start = self._busy_since[name]
                    if start is not None:
                        self._completed_durations.append(max(now - start, 0.0))
                    self._busy_since[name] = None
                    self._tasks_completed[name] += 1
                self._last_state[name] = state

    def _publish_summary(self):
        out = FleetMetrics()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.robot_names = list(self.robot_names)
        with self._lock:
            out.distance_travelled_m = [
                float(self._distance_m[r]) for r in self.robot_names
            ]
            out.tasks_completed = [
                int(self._tasks_completed[r]) for r in self.robot_names
            ]
            if self._completed_durations:
                out.mean_seconds_per_task = float(
                    sum(self._completed_durations) / len(self._completed_durations)
                )
            else:
                out.mean_seconds_per_task = 0.0
            out.total_tasks_dispatched = int(self._total_dispatched)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FleetMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
