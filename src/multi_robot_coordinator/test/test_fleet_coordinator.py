"""Integration tests for FleetCoordinator.

These tests exercise the real coordinator node with mocked
/{robot}/navigate_to_pose action servers, so they run without Gazebo or
Nav2. A MultiThreadedExecutor spins the coordinator + mocks + a small test
harness node inside the test process.

The mock action servers and the test-harness node are module-scoped —
only the FleetCoordinator is rebuilt per test. This avoids DDS discovery
races where a freshly-started test picks up a still-advertised action
server from the previous test and receives each goal twice.

Run directly:
    pytest src/multi_robot_coordinator/test/test_fleet_coordinator.py -v
"""

import threading
import time
from typing import List, Optional

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from multi_robot_coordinator.fleet_coordinator import (
    STATE_BUSY,
    STATE_IDLE,
    FleetCoordinator,
)
from multi_robot_coordinator_interfaces.msg import FleetStatus
from multi_robot_coordinator_interfaces.srv import SendGoal


class MockNav2Server(Node):
    """Minimal NavigateToPose action server with a switchable outcome."""

    def __init__(self, robot_name: str):
        super().__init__(f"mock_nav_{robot_name}")
        self._cfg_lock = threading.Lock()
        self.outcome = "succeed"  # "succeed" | "abort"
        self.execute_delay = 1.0
        self.goals_received = 0
        self._server = ActionServer(
            self,
            NavigateToPose,
            f"/{robot_name}/navigate_to_pose",
            execute_callback=self._execute,
        )

    def configure(self, outcome: str, execute_delay: float):
        with self._cfg_lock:
            self.outcome = outcome
            self.execute_delay = execute_delay

    def reset(self):
        with self._cfg_lock:
            self.goals_received = 0
            self.outcome = "succeed"
            self.execute_delay = 1.0

    def _execute(self, goal_handle):
        with self._cfg_lock:
            self.goals_received += 1
            outcome = self.outcome
            delay = self.execute_delay
        time.sleep(delay)
        result = NavigateToPose.Result()
        if outcome == "succeed":
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


class Harness(Node):
    """Test-side client for SendGoal + collector for FleetStatus."""

    def __init__(self):
        super().__init__("test_harness")
        self.latest: Optional[FleetStatus] = None
        self.history: List[FleetStatus] = []
        self.create_subscription(
            FleetStatus, "/fleet_coordinator/status", self._status_cb, 10
        )
        self._client = self.create_client(SendGoal, "/fleet_coordinator/send_goal")

    def _status_cb(self, msg: FleetStatus):
        self.latest = msg
        self.history.append(msg)

    def reset(self):
        self.latest = None
        self.history.clear()

    def send(self, x: float, y: float, timeout: float = 5.0) -> SendGoal.Response:
        assert self._client.wait_for_service(timeout_sec=timeout), \
            "SendGoal service not available"
        req = SendGoal.Request()
        req.x = float(x)
        req.y = float(y)
        req.yaw = 0.0
        fut = self._client.call_async(req)
        end = time.time() + timeout
        while time.time() < end:
            if fut.done():
                return fut.result()
            time.sleep(0.02)
        raise TimeoutError("SendGoal call did not complete")


def wait_until(predicate, timeout: float = 10.0, tick: float = 0.05) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(tick)
    return False


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def fleet_env():
    """Long-lived mocks + harness + spinning executor.

    Persisting these across tests is what prevents DDS from routing goals
    to a still-advertised server from a previous test.
    """
    robots = ["robot_1", "robot_2"]
    mocks = {r: MockNav2Server(r) for r in robots}
    h = Harness()

    exe = MultiThreadedExecutor(num_threads=8)
    exe.add_node(h)
    for m in mocks.values():
        exe.add_node(m)

    stop = threading.Event()

    def _spin():
        while not stop.is_set() and rclpy.ok():
            exe.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    time.sleep(0.4)

    try:
        yield mocks, h, exe
    finally:
        stop.set()
        thread.join(timeout=3.0)
        h.destroy_node()
        for m in mocks.values():
            m.destroy_node()


@pytest.fixture
def harness(fleet_env):
    """Fresh FleetCoordinator per test; shared mocks + harness."""
    mocks, h, exe = fleet_env

    for m in mocks.values():
        m.reset()
    h.reset()

    coord = FleetCoordinator()
    exe.add_node(coord)
    # Give the new coordinator time to register its service + action clients.
    time.sleep(0.4)

    try:
        yield coord, mocks, h
    finally:
        exe.remove_node(coord)
        coord.destroy_node()
        # Brief settle so DDS entities unwind before the next coordinator.
        time.sleep(0.3)


def test_service_accepts_goal_and_returns_task_id(harness):
    _, mocks, t = harness
    for m in mocks.values():
        m.configure(outcome="succeed", execute_delay=0.2)

    resp = t.send(1.0, 0.5)
    assert resp.success is True
    assert resp.task_id.startswith("task_")


def test_successful_goal_drains_queue_and_flips_to_idle(harness):
    _, mocks, t = harness
    for m in mocks.values():
        m.configure(outcome="succeed", execute_delay=0.3)

    t.send(1.0, 0.0)

    assert wait_until(
        lambda: sum(m.goals_received for m in mocks.values()) >= 1,
        timeout=5.0,
    ), "no mock server received the goal"

    assert wait_until(
        lambda: t.latest is not None
        and all(s == STATE_IDLE for s in t.latest.robot_states)
        and t.latest.queue_size == 0,
        timeout=5.0,
    ), f"robots never returned to idle; last status={t.latest}"


def test_two_goals_reach_both_robots(harness):
    _, mocks, t = harness
    # Long-enough delay that the first robot is still busy when the second
    # dispatch tick fires, so the second goal has to go to the other robot.
    for m in mocks.values():
        m.configure(outcome="succeed", execute_delay=1.5)

    t.send(1.0, 0.0)
    t.send(-1.0, 0.0)

    assert wait_until(
        lambda: all(m.goals_received >= 1 for m in mocks.values()),
        timeout=8.0,
    ), (
        "goals did not reach both robots: "
        f"{ {r: m.goals_received for r, m in mocks.items()} }"
    )

    assert wait_until(
        lambda: t.latest is not None
        and all(s == STATE_IDLE for s in t.latest.robot_states)
        and t.latest.queue_size == 0,
        timeout=10.0,
    )


def test_aborted_goal_is_retried_once_then_dropped(harness):
    _, mocks, t = harness
    for m in mocks.values():
        m.configure(outcome="abort", execute_delay=0.2)

    t.send(5.0, 5.0)

    # max_retries=1 → expect exactly 2 attempts across both mocks.
    assert wait_until(
        lambda: sum(m.goals_received for m in mocks.values()) >= 2,
        timeout=8.0,
    ), (
        "expected 2 attempts; got "
        f"{ {r: m.goals_received for r, m in mocks.items()} }"
    )

    assert wait_until(
        lambda: t.latest is not None
        and all(s == STATE_IDLE for s in t.latest.robot_states)
        and t.latest.queue_size == 0,
        timeout=5.0,
    )

    time.sleep(1.0)
    assert sum(m.goals_received for m in mocks.values()) == 2


def test_third_goal_stays_queued_while_both_robots_busy(harness):
    _, mocks, t = harness
    for m in mocks.values():
        m.configure(outcome="succeed", execute_delay=2.0)

    t.send(1.0, 0.0)
    t.send(-1.0, 0.0)
    t.send(0.0, 1.0)

    assert wait_until(
        lambda: t.latest is not None
        and all(s == STATE_BUSY for s in t.latest.robot_states)
        and t.latest.queue_size >= 1,
        timeout=6.0,
    ), f"never observed both-busy + queue>=1; last status={t.latest}"

    assert wait_until(
        lambda: t.latest is not None
        and all(s == STATE_IDLE for s in t.latest.robot_states)
        and t.latest.queue_size == 0,
        timeout=15.0,
    )
