import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point # Required backend driver for point transformations
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import math

from vlm_nav_robot_interfaces.srv import FindAndGo

class HybridGrounder(Node):
    def __init__(self):
        super().__init__('hybrid_grounder')
        
        # Threshold and Frame Configurations
        sim_time_param = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

        self.declare_parameter('high_confidence_thresh', 0.85)
        self.declare_parameter('low_confidence_thresh', 0.40)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('standoff_m', 0.4) # Stop 40cm short of hitting the asset surface
        
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._cv_image = None
        self._depth_image = None
        self._camera_info = None
        self._latest_bbox = None
        self._latest_label = ""
        
        cb_group = ReentrantCallbackGroup()
        
        # TF2 Frame Listeners
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Nav2 Action Client Interface
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=cb_group)
        
        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 1, callback_group=cb_group)
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 1, callback_group=cb_group)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 1, callback_group=cb_group)
        
        # Service Server
        self.srv = self.create_service(FindAndGo, '~/find_and_go', self._handle_request, callback_group=cb_group)
        
        # GUI Update Loop
        self.gui_timer = self.create_timer(0.03, self._refresh_gui)
        self.get_logger().info("Hybrid Navigation Grounder Node fully configured.")

    def _image_cb(self, msg):
        with self._lock:
            self._cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        with self._lock:
            if msg.encoding == '32FC1':
                self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding == '16UC1':
                self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32) / 1000.0

    def _info_cb(self, msg):
        with self._lock:
            self._camera_info = msg

    def _refresh_gui(self):
        with self._lock:
            if self._cv_image is None:
                return
            display_img = self._cv_image.copy()
            bbox = self._latest_bbox
            label = self._latest_label

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Hybrid Grounder Test View", display_img)
        cv2.waitKey(1)

    def _detect_color_target(self, cv_img, target_name):
        color_registry = {
            "red": {
                "lower1": np.array([0, 70, 50]),   "upper1": np.array([10, 255, 255]),
                "lower2": np.array([165, 70, 50]), "upper2": np.array([180, 255, 255]),
                "ideal_hue": 0
            },
            "green": {
                "lower1": np.array([35, 70, 50]),  "upper1": np.array([85, 255, 255]),
                "ideal_hue": 60
            },
            "blue": {
                "lower1": np.array([100, 70, 50]), "upper1": np.array([140, 255, 255]),
                "ideal_hue": 120
            },
            "yellow": {
                "lower1": np.array([20, 70, 50]),  "upper1": np.array([35, 255, 255]),
                "ideal_hue": 30
            }
        }

        parsed_color = None
        for color in color_registry.keys():
            if color in target_name.lower():
                parsed_color = color
                break

        if not parsed_color:
            return None, 0.0

        cfg = color_registry[parsed_color]
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, cfg["lower1"], cfg["upper1"])
        if "lower2" in cfg:
            mask2 = cv2.inRange(hsv_img, cfg["lower2"], cfg["upper2"])
            mask = cv2.bitwise_or(mask, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 400:
            return None, 0.0

        x, y, w, h = cv2.boundingRect(largest_contour)
        bbox = [x, y, x + w, y + h]

        roi = hsv_img[y:y+h, x:x+w]
        avg_hue = np.median(roi[:, :, 0])
        avg_val = np.median(roi[:, :, 2])

        hue_error = min(abs(avg_hue - cfg["ideal_hue"]), 180 - abs(avg_hue - cfg["ideal_hue"]))
        hue_confidence = max(0.0, 1.0 - (hue_error / 20.0))
        lighting_penalty = avg_val / 255.0
        
        final_confidence = float(0.8 * hue_confidence + 0.2 * lighting_penalty)
        return bbox, round(final_confidence, 2)

    def _sample_depth(self, depth_arr, x1, y1, x2, y2):
        h, w = depth_arr.shape
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        
        bw, bh = x2 - x1, y2 - y1
        sx = bw // 5
        ix1, ix2 = x1 + sx, x2 - sx
        iy1, iy2 = y1, y1 + (bh * 6) // 10
        
        patch = depth_arr[iy1:iy2, ix1:ix2]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            full = depth_arr[y1:y2, x1:x2]
            valid = full[np.isfinite(full) & (full > 0.05)]
            if valid.size == 0:
                return None
        return float(np.median(valid))

    def _handle_request(self, request, response):
        target = request.target
        self.get_logger().info(f"Received target request: '{target}'")
        
        with self._lock:
            img = self._cv_image.copy() if self._cv_image is not None else None
            depth = self._depth_image.copy() if self._depth_image is not None else None
            info = self._camera_info
            
        if img is None or depth is None or info is None:
            response.success = False
            response.reason = "Sensor streams unavailable."
            return response
            
        # Execute Detection Engine
        bbox, confidence = self._detect_color_target(img, target)
        
        with self._lock:
            self._latest_bbox = bbox
            self._latest_label = f"{target}: {confidence*100:.1f}%" if bbox else ""

        high_thresh = self.get_parameter('high_confidence_thresh').value
        low_thresh = self.get_parameter('low_confidence_thresh').value
        
        if confidence < low_thresh:
            response.success = False
            response.reason = f"Target '{target}' not detected locally."
            return response
            
        if low_thresh <= confidence < high_thresh:
            self.get_logger().warning(f"Uncertain match ({confidence*100:.1f}%). Waiting verification...")
            user_approval = input(f"-> Target '{target}' matched at {confidence*100:.1f}%. Confirm execution? (y/n): ")
            if user_approval.lower() != 'y':
                response.success = False
                response.reason = "Aborted by user."
                return response

        # Resolve Spatial Depth Logic
        x1, y1, x2, y2 = bbox
        d = self._sample_depth(depth, x1, y1, x2, y2)
        if d is None:
            response.success = False
            response.reason = "Failed to pull valid distance data."
            return response

        # Compute Pinhole Back-Projection Matrix 
        u = int(round((x1 + x2) / 2))
        v = int(round((y1 + y2) / 2))
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        Z = d

        # Create PointStamped relative to camera frame
        camera_point = PointStamped()
        camera_point.header.stamp = self.get_clock().now().to_msg()
        camera_point.header.frame_id = info.header.frame_id or "camera_optical_link"
        camera_point.point.x = float(X)
        camera_point.point.y = float(Y)
        camera_point.point.z = float(Z)

        # ─── TRANSLATE COORDINATES TO THE STABLE WORLD FRAME (MAP) ───
        goal_frame = self.get_parameter('goal_frame').value
        base_frame = self.get_parameter('robot_base_frame').value
        
        try:
            # Transform target point into map frame
            world_point = self._tf_buffer.transform(
                camera_point, goal_frame, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Find where the robot base footprint is currently in the map frame
            base_transform = self._tf_buffer.lookup_transform(
                goal_frame, base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as e:
            response.success = False
            response.reason = f"TF2 coordinate frame frame lookup failed: {e}"
            return response

        # Extract robot base positions
        rx = base_transform.transform.translation.x
        ry = base_transform.transform.translation.y
        
        # Distance and heading vectors from robot to object
        dx = world_point.point.x - rx
        dy = world_point.point.y - ry
        target_dist = math.hypot(dx, dy)

        # Calculate Standoff Waypoint positioning
        standoff = self.get_parameter('standoff_m').value
        scaling_factor = max(0.0, (target_dist - standoff) / target_dist)
        
        goal_x = rx + dx * scaling_factor
        goal_y = ry + dy * scaling_factor
        yaw_heading = math.atan2(dy, dx)

        # Format final PoseStamped navigation message
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = goal_frame
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw_heading / 2), w=math.cos(yaw_heading / 2)
        )

        # ─── DISPATCH DRIVE COMMANDS TO NAV2 ACTION SERVER ───
        if self._nav_client.wait_for_server(timeout_sec=1.0):
            action_goal = NavigateToPose.Goal()
            action_goal.pose = goal_pose
            self._nav_client.send_goal_async(action_goal)
            status_text = "Goal dispatched cleanly to Nav2 action stack."
        else:
            status_text = "Nav2 action server offline — coordinates generated only."

        response.success = True
        response.reason = f"Target located at distance {d:.2f}m. {status_text}"
        response.goal = goal_pose
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HybridGrounder()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()