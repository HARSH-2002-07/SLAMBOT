"""Subscribe to /camera/image_raw, save one frame, exit.

Usage (with the sim running):
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/vlm_nav_robot/tools/save_frame.py                    # → /tmp/gazebo_frame.png
    python3 src/vlm_nav_robot/tools/save_frame.py /tmp/out.png       # custom path
    python3 src/vlm_nav_robot/tools/save_frame.py /tmp/out.png /robot_1/camera/image_raw

Designed for the Gemini preflight on real sim frames — hands the output to
gemini_probe.py. No cv_bridge dependency; decodes rgb8/bgr8 directly.
"""

import sys
import time

import numpy as np
import PIL.Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_OUT = "/tmp/gazebo_frame.png"
TIMEOUT_S = 10.0


class FrameSaver(Node):
    def __init__(self, topic: str, out_path: str) -> None:
        super().__init__("save_frame")
        self.out_path = out_path
        self.frame: PIL.Image.Image | None = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, topic, self._cb, qos)
        self.get_logger().info(f"waiting for a frame on {topic} …")

    def _cb(self, msg: Image) -> None:
        if self.frame is not None:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        elif msg.encoding != "rgb8":
            self.get_logger().error(f"unsupported encoding: {msg.encoding}")
            return
        self.frame = PIL.Image.fromarray(arr, mode="RGB")
        self.frame.save(self.out_path)
        self.get_logger().info(
            f"saved {msg.width}x{msg.height} → {self.out_path}"
        )


def main() -> int:
    out_path = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_OUT
    topic = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_TOPIC

    rclpy.init()
    node = FrameSaver(topic, out_path)
    deadline = time.time() + TIMEOUT_S
    try:
        while rclpy.ok() and node.frame is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.frame is None:
        print(f"ERROR: no frame on {topic} within {TIMEOUT_S:.0f}s",
              file=sys.stderr)
        return 1
    print(out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
