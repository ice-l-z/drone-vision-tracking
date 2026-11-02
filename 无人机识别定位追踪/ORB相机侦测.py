import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_RATE_HZ = 30.0

TEMPLATE_PATH = "iamge.jpg"

ORB_NFEATURES = 1500
RATIO_THRESH = 0.75
MIN_MATCH_COUNT = 20
RANSAC_THRESH = 5.0
DETECT_SCALE = 1.0

FX = 539.936
FY = 539.936
CX = 640.0
CY = 480.0
CAMERA_MATRIX = None
DIST_COEFFS = None

R_B_C = np.array([[0.0, -1.0, 0.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])
T_B_C = np.array([0.0, 0.0, 0.0])

GROUND_Z = 0.0
FRAME_ID = "map"

DISPLAY_ENABLED = True

PUBLISH_RATE_HZ = 10.0
TARGET_STALE_TIMEOUT = 1.0


def undistort(frame):
    if CAMERA_MATRIX is None or DIST_COEFFS is None:
        return frame
    h, w = frame.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
    return cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS, None, new_mtx)


class OrbObjectDetector(Node):
    def __init__(self):
        super().__init__('orb_object_detector')

        self.orb = cv2.ORB_create(nfeatures=ORB_NFEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.template = None
        self.template_gray = None
        self.template_kp = None
        self.template_des = None
        self._load_template(TEMPLATE_PATH)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if self.cap.isOpened():
            if FRAME_WIDTH is not None:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            if FRAME_HEIGHT is not None:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.ned_pub = self.create_publisher(PointStamped, '/object_position_ned', 10)
        self.bearing_pub = self.create_publisher(Float32, '/object/bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/object/distance', 10)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, qos)

        self.position_ned = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.position_valid = False
        self.attitude_valid = False

        self.last_center = None
        self.last_corners = None
        self.last_ned = None
        self.last_bearing_rel = None
        self.last_bearing_north = None
        self.last_distance_h = None
        self.last_distance_3d = None
        self.last_matches = 0

        self._last_detection_time = None

        self.cam_timer = self.create_timer(1.0 / CAMERA_RATE_HZ, self.camera_callback)
        self.pub_timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_continuous)

    def _compute_template_features(self, bgr):
        self.template = bgr
        self.template_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(self.template_gray, None)
        self.template_kp, self.template_des = kp, des

    def _load_template(self, path):
        if not os.path.exists(path):
            return
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self._compute_template_features(bgr)

    def local_position_callback(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.position_ned = np.array([msg.x, msg.y, msg.z])
            self.position_valid = True
        else:
            self.position_valid = False

    def attitude_callback(self, msg):
        self.quat = np.array(msg.q, dtype=np.float64)
        self.attitude_valid = True

    def camera_callback(self):
        if not self.cap.isOpened():
            return
        ok, frame = self.cap.read()
        if not ok:
            return
        self.process_frame(frame)

    def process_frame(self, frame):
        if self.template_des is None:
            return

        frame = undistort(frame)

        if abs(DETECT_SCALE - 1.0) > 1e-6:
            h, w = frame.shape[:2]
            small = cv2.resize(frame, (int(w * DETECT_SCALE), int(h * DETECT_SCALE)),
                               interpolation=cv2.INTER_AREA)
        else:
            small = frame

        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        kp2, des2 = self.orb.detectAndCompute(gray, None)

        corners = None
        center = None
        matches_count = 0

        if des2 is not None and len(kp2) >= 4:
            knn = self.bf.knnMatch(self.template_des, des2, k=2)
            good = [m for m, n in knn if m.distance < RATIO_THRESH * n.distance]
            matches_count = len(good)

            if len(good) >= MIN_MATCH_COUNT:
                src_pts = np.float32([self.template_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, RANSAC_THRESH)
                if H is not None:
                    th, tw = self.template_gray.shape[:2]
                    tc = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]]).reshape(-1, 1, 2)
                    corners = cv2.perspectiveTransform(tc, H).reshape(-1, 2)
                    center = corners.mean(axis=0)
                    if abs(DETECT_SCALE - 1.0) > 1e-6:
                        corners = corners / DETECT_SCALE
                        center = center / DETECT_SCALE

        ned = None
        if center is not None:
            ned = self.pixel_to_ned(center[0], center[1])

        self._update_result(center, corners, ned, matches_count)
        self.display_result(frame, corners, center, ned, matches_count)

    def _update_result(self, center, corners, ned, matches_count):
        self.last_center = center
        self.last_corners = corners
        self.last_matches = matches_count

        if ned is None:
            return

        self.last_ned = ned

        rel = np.array(ned) - self.position_ned
        distance_h = float(np.hypot(rel[0], rel[1]))
        distance_3d = float(np.linalg.norm(rel))

        bearing_north = float(np.degrees(np.arctan2(rel[1], rel[0]))) % 360.0

        R_w_b = self.quat2rot(self.quat)
        fwd = R_w_b @ np.array([1.0, 0.0, 0.0])
        theta_drone = np.arctan2(fwd[1], fwd[0])
        theta_target = np.arctan2(rel[1], rel[0])
        dtheta = theta_target - theta_drone
        bearing_rel = float(np.degrees(np.arctan2(np.sin(dtheta), np.cos(dtheta))))

        self.last_bearing_rel = bearing_rel
        self.last_bearing_north = bearing_north
        self.last_distance_h = distance_h
        self.last_distance_3d = distance_3d

        self._last_detection_time = self.get_clock().now()

    @staticmethod
    def quat2rot(q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def pixel_to_ned(self, u, v):
        if not (self.position_valid and self.attitude_valid):
            return None

        p_cam = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])

        p_b = R_B_C @ p_cam
        t_b = T_B_C

        R_w_b = self.quat2rot(self.quat)
        A = R_w_b @ p_b
        B = R_w_b @ t_b + self.position_ned

        if abs(A[2]) < 1e-6:
            return None
        depth = (GROUND_Z - B[2]) / A[2]
        if depth <= 0:
            return None

        return (depth * A[0] + B[0],
                depth * A[1] + B[1],
                GROUND_Z)

    def publish_ned(self, ned):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.point.x = float(ned[0])
        msg.point.y = float(ned[1])
        msg.point.z = float(ned[2])
        self.ned_pub.publish(msg)

    def publish_bearing(self, bearing_rel):
        msg = Float32()
        msg.data = float(bearing_rel)
        self.bearing_pub.publish(msg)

    def publish_distance(self, distance_h):
        msg = Float32()
        msg.data = float(distance_h)
        self.distance_pub.publish(msg)

    def publish_continuous(self):
        if self.last_ned is None or self._last_detection_time is None:
            return
        dt = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
        if dt > TARGET_STALE_TIMEOUT:
            self.last_ned = None
            return
        self.publish_ned(self.last_ned)
        if self.last_bearing_rel is not None:
            self.publish_bearing(self.last_bearing_rel)
        if self.last_distance_h is not None:
            self.publish_distance(self.last_distance_h)

    def display_result(self, frame, corners, center, ned, matches_count):
        if not DISPLAY_ENABLED:
            return
        out = frame.copy()
        if corners is not None:
            pts = corners.astype(np.int32)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        if center is not None:
            c = tuple(np.round(center).astype(int))
            cv2.circle(out, c, 5, (0, 0, 255), -1)
            cv2.cross(out, c, (255, 0, 0), 2)

        lines = []
        if self.last_bearing_rel is not None:
            lines.append(f"rel:{self.last_bearing_rel:+.1f}deg  N:{self.last_bearing_north:.1f}deg")
        if self.last_distance_h is not None:
            lines.append(f"distH:{self.last_distance_h:.2f}m  dist3d:{self.last_distance_3d:.2f}m")
        if ned is not None:
            lines.append(f"NED x:{ned[0]:.2f} y:{ned[1]:.2f}")
        lines.append(f"matches:{matches_count}")
        for i, txt in enumerate(lines):
            cv2.putText(out, txt, (10, 30 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        try:
            cv2.imshow("ORB target (press 'q' to quit)", out)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            raise
        except Exception:
            pass

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
