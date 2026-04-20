"""VLM-grounded navigation node.

On FindAndGo request, this node:
  1. Grabs the latest RGB + depth frame + camera_info
  2. Asks Gemini to ground the target in the image (schema-validated JSON)
  3. Back-projects the bbox centre pixel through depth to a 3D point
     in the optical frame
  4. tf2-transforms that point into the configured goal frame (default: odom)
  5. Offsets the goal by `standoff_m` along robot→target so Nav2 doesn't
     plan into the object's surface
  6. Sends a NavigateToPose goal (best-effort — action server optional)
     and returns the computed PoseStamped via the service
"""

import json
import math
import os
import threading
import time

import numpy as np
import PIL.Image
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from google import genai
from google.genai import errors as genai_errors
from google.genai import types
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point  # noqa: F401 — registers PointStamped for tf2

from vlm_nav_robot_interfaces.srv import FindAndGo


MODEL_NAME = "gemini-2.5-flash"
RETRY_STATUSES = (429, 503)
MAX_ATTEMPTS = 3

PROMPT_TEMPLATE = (
    "Detect the 2D bounding box of the single best match for this target "
    "in the image: {target!r}. "
    "If the target is not visible, set confidence to 0.0 and box_2d to "
    "[0, 0, 0, 0]."
)

RESPONSE_SCHEMA = {
    "type": "OBJECT",
    "properties": {
        "label": {"type": "STRING"},
        "box_2d": {
            "type": "ARRAY",
            "items": {"type": "INTEGER"},
            "minItems": 4,
            "maxItems": 4,
        },
        "confidence": {"type": "NUMBER"},
        "notes": {"type": "STRING"},
    },
    "required": ["label", "box_2d", "confidence"],
}


class VLMGrounder(Node):
    def __init__(self) -> None:
        super().__init__("vlm_grounder")

        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("standoff_m", 0.3)
        self.declare_parameter("goal_frame", "odom")
        self.declare_parameter("robot_base_frame", "base_footprint")

        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY env var is not set")
        self._client = genai.Client(api_key=api_key)

        self._lock = threading.Lock()
        self._rgb: Image | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None

        cb = ReentrantCallbackGroup()

        self.create_subscription(Image, "/camera/image_raw",
                                 self._rgb_cb, 1, callback_group=cb)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._depth_cb, 1, callback_group=cb)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._info_cb, 1, callback_group=cb)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=cb
        )

        self._service = self.create_service(
            FindAndGo, "~/find_and_go", self._on_request, callback_group=cb
        )
        self.get_logger().info("vlm_grounder ready")

    def _rgb_cb(self, msg: Image) -> None:
        with self._lock:
            self._rgb = msg

    def _depth_cb(self, msg: Image) -> None:
        with self._lock:
            self._depth = msg

    def _info_cb(self, msg: CameraInfo) -> None:
        with self._lock:
            self._info = msg

    def _on_request(self, request: FindAndGo.Request,
                    response: FindAndGo.Response) -> FindAndGo.Response:
        with self._lock:
            rgb, depth, info = self._rgb, self._depth, self._info

        if rgb is None or depth is None or info is None:
            return self._fail(response, "no RGB/depth/camera_info received yet")

        pil = self._decode_rgb(rgb)
        if pil is None:
            return self._fail(response,
                              f"unsupported RGB encoding: {rgb.encoding}")

        try:
            parsed = self._gemini_detect(pil, request.target)
        except Exception as e:
            return self._fail(response, f"Gemini call failed: {e}")

        conf_thresh = self.get_parameter(
            "confidence_threshold").get_parameter_value().double_value
        if parsed["confidence"] < conf_thresh:
            return self._fail(
                response,
                f"low confidence {parsed['confidence']:.2f} for "
                f"'{request.target}' — target may not be visible",
            )

        y1, x1, y2, x2 = parsed["box_2d"]
        # Pixel bbox (for logging / debugging)
        px_x1 = int(round(x1 / 1000 * rgb.width))
        px_y1 = int(round(y1 / 1000 * rgb.height))
        px_x2 = int(round(x2 / 1000 * rgb.width))
        px_y2 = int(round(y2 / 1000 * rgb.height))
        u = int(round((x1 + x2) / 2 / 1000 * rgb.width))
        v = int(round((y1 + y2) / 2 / 1000 * rgb.height))

        depth_arr = self._decode_depth(depth)
        if depth_arr is None:
            return self._fail(response,
                              f"unsupported depth encoding: {depth.encoding}")

        d = self._sample_depth(depth_arr, px_x1, px_y1, px_x2, px_y2)
        if d is None:
            return self._fail(
                response,
                f"no valid depth in bbox ({px_x1},{px_y1},{px_x2},{px_y2}) — "
                "target may be outside depth range or occluded",
            )

        self.get_logger().info(
            f"[debug] bbox_px=({px_x1},{px_y1},{px_x2},{px_y2}) "
            f"centre=({u},{v}) depth={d:.3f}m "
            f"img={rgb.width}x{rgb.height} "
            f"K=[fx={info.k[0]:.1f} fy={info.k[4]:.1f} "
            f"cx={info.k[2]:.1f} cy={info.k[5]:.1f}] "
            f"optical_frame={info.header.frame_id!r}"
        )

        # Intrinsics from camera_info.k (row-major 3x3)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        Z = d

        point = PointStamped()
        # Stamp 0 → tf2 uses latest transform. Safe here because the robot is
        # stationary during a FindAndGo call (Gemini adds multi-second latency,
        # so the image timestamp is typically older than the TF buffer).
        point.header.stamp = rclpy.time.Time().to_msg()
        point.header.frame_id = info.header.frame_id or "camera_optical_link"
        point.point.x = float(X)
        point.point.y = float(Y)
        point.point.z = float(Z)

        goal_frame = self.get_parameter(
            "goal_frame").get_parameter_value().string_value
        try:
            point_world = self._tf_buffer.transform(
                point, goal_frame,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as e:
            return self._fail(
                response,
                f"tf2 transform {point.header.frame_id} → {goal_frame} "
                f"failed: {e}",
            )

        base_frame = self.get_parameter(
            "robot_base_frame").get_parameter_value().string_value
        try:
            base_tf = self._tf_buffer.lookup_transform(
                goal_frame, base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as e:
            return self._fail(response,
                              f"tf2 lookup {goal_frame}←{base_frame} failed: {e}")

        rx = base_tf.transform.translation.x
        ry = base_tf.transform.translation.y
        dx = point_world.point.x - rx
        dy = point_world.point.y - ry
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f"[debug] optical_xyz=({X:.3f},{Y:.3f},{Z:.3f}) → "
            f"{goal_frame}=({point_world.point.x:.3f},"
            f"{point_world.point.y:.3f},{point_world.point.z:.3f}) "
            f"robot@({rx:.3f},{ry:.3f}) "
            f"target_dist={dist:.3f}m"
        )
        if dist < 1e-3:
            return self._fail(response, "target coincides with robot position")

        standoff = self.get_parameter(
            "standoff_m").get_parameter_value().double_value
        frac = max(0.0, (dist - standoff) / dist)
        goal_x = rx + dx * frac
        goal_y = ry + dy * frac
        yaw = math.atan2(dy, dx)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = goal_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
        )

        if self._nav_client.wait_for_server(timeout_sec=1.0):
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal
            self._nav_client.send_goal_async(nav_goal)
            sent = "sent to NavigateToPose"
        else:
            sent = "NavigateToPose server unavailable — pose returned only"

        response.success = True
        response.reason = (
            f"target='{parsed['label']}' conf={parsed['confidence']:.2f} "
            f"goal=({goal_x:.2f},{goal_y:.2f}) yaw={yaw:.2f} [{sent}]"
        )
        response.goal = goal
        self.get_logger().info(response.reason)
        return response

    # ─── helpers ────────────────────────────────────────────────────

    @staticmethod
    def _decode_rgb(msg: Image) -> PIL.Image.Image | None:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        elif msg.encoding != "rgb8":
            return None
        return PIL.Image.fromarray(arr, mode="RGB")

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray | None:
        if msg.encoding == "32FC1":
            return np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
        if msg.encoding == "16UC1":
            # mm → m
            return (
                np.frombuffer(msg.data, dtype=np.uint16)
                .reshape(msg.height, msg.width)
                .astype(np.float32)
                / 1000.0
            )
        return None

    @staticmethod
    def _sample_depth(depth_arr: np.ndarray, x1: int, y1: int,
                      x2: int, y2: int) -> float | None:
        # Sample the upper 60% of the bbox (shrunk 20% horizontally) and
        # return the median depth. For ground-sitting objects the lower
        # portion of Gemini's bbox typically leaks onto the floor just in
        # front of the target — that floor is *closer* than the target, so
        # naïve "closest surface" heuristics read the floor and the robot
        # stops early. The upper portion is reliably on the object face.
        h, w = depth_arr.shape
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        bw, bh = x2 - x1, y2 - y1
        sx = bw // 5
        ix1, ix2 = x1 + sx, x2 - sx
        iy1, iy2 = y1, y1 + (bh * 6) // 10
        if ix2 <= ix1 or iy2 <= iy1:
            ix1, iy1, ix2, iy2 = x1, y1, x2, y2
        patch = depth_arr[iy1:iy2, ix1:ix2]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _gemini_detect(self, pil_img: PIL.Image.Image,
                       target: str) -> dict:
        prompt = PROMPT_TEMPLATE.format(target=target)
        for attempt in range(1, MAX_ATTEMPTS + 1):
            try:
                resp = self._client.models.generate_content(
                    model=MODEL_NAME,
                    contents=[pil_img, prompt],
                    config=types.GenerateContentConfig(
                        response_mime_type="application/json",
                        response_schema=RESPONSE_SCHEMA,
                    ),
                )
                return json.loads(resp.text)
            except genai_errors.APIError as e:
                if e.code in RETRY_STATUSES and attempt < MAX_ATTEMPTS:
                    backoff = 2 ** attempt
                    self.get_logger().warning(
                        f"Gemini {e.code}; retrying in {backoff}s "
                        f"(attempt {attempt}/{MAX_ATTEMPTS})"
                    )
                    time.sleep(backoff)
                    continue
                raise
        raise RuntimeError("unreachable")

    def _fail(self, response: FindAndGo.Response,
              reason: str) -> FindAndGo.Response:
        response.success = False
        response.reason = reason
        self.get_logger().warning(reason)
        return response


def main() -> None:
    rclpy.init()
    node = VLMGrounder()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
