"""ArUco fiducial detector.

Subscribes to the OAK-D RGB stream + camera_info and publishes a JSON record
for every detected marker. Each landmark in the tour is identified by an
ArUco ID; the executive uses these detections to confirm POI arrival and
trigger narration.
"""

import json
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

ARUCO_DICTS = {
    '4x4_50':    cv2.aruco.DICT_4X4_50,
    '4x4_100':   cv2.aruco.DICT_4X4_100,
    '4x4_250':   cv2.aruco.DICT_4X4_250,
    '4x4_1000':  cv2.aruco.DICT_4X4_1000,
    '5x5_50':    cv2.aruco.DICT_5X5_50,
    '5x5_100':   cv2.aruco.DICT_5X5_100,
    '5x5_250':   cv2.aruco.DICT_5X5_250,
    '6x6_250':   cv2.aruco.DICT_6X6_250,
    '7x7_250':   cv2.aruco.DICT_7X7_250,
}


@dataclass
class Detection:
    marker_id: int
    distance_m: float
    bearing_rad: float
    pose_camera: tuple   # (x, y, z) in camera frame, metres
    corners_px: list


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('dictionary', '4x4_50')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('min_detection_distance_m', 0.20)
        self.declare_parameter('max_detection_distance_m', 3.50)

        dict_name = self.get_parameter('dictionary').value
        if dict_name not in ARUCO_DICTS:
            self.get_logger().warn(f'Unknown dictionary "{dict_name}", falling back to 4x4_50.')
            dict_name = '4x4_50'

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        # Build DetectorParameters in a way that works on both OpenCV 4.6 and 4.7+.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
        # The new cv2.aruco.ArucoDetector class is known to SIGSEGV on first call
        # in some OpenCV 4.7 builds; the free-function form is more stable across
        # versions, so we always use it.
        self.detector = None

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.min_d = float(self.get_parameter('min_detection_distance_m').value)
        self.max_d = float(self.get_parameter('max_detection_distance_m').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.on_image,
            sensor_qos,
        )

        self.detect_pub = self.create_publisher(String, '/landmarks/detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/landmarks/annotated', 1) \
            if self.get_parameter('publish_annotated').value else None

        self.get_logger().info(
            f'ArUco detector ready (dict={dict_name}, marker_size={self.marker_size:.3f} m).'
        )

    def on_camera_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def on_image(self, msg: Image):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failure: {exc}')
            return

        # Force contiguous memory: non-contiguous arrays from cv_bridge are a
        # documented source of SIGSEGVs inside OpenCV native code.
        frame = np.ascontiguousarray(frame)
        gray = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)
        except cv2.error as exc:
            self.get_logger().warn(f'detectMarkers raised cv2.error: {exc}')
            return

        if ids is None or len(ids) == 0:
            return

        detections = self._estimate_poses(corners, ids)
        for det in detections:
            if not (self.min_d <= det.distance_m <= self.max_d):
                continue
            self.detect_pub.publish(String(data=json.dumps({
                'id': det.marker_id,
                'distance_m': det.distance_m,
                'bearing_rad': det.bearing_rad,
                'pose_camera': det.pose_camera,
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
            })))

        if self.annotated_pub is not None:
            try:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            except cv2.error as exc:
                self.get_logger().warn(f'drawDetectedMarkers failed: {exc}')

    def _estimate_poses(self, corners, ids):
        """Estimate per-marker pose using solvePnP on the marker's corner template."""
        half = self.marker_size / 2.0
        # Marker corner order matches cv2.aruco: TL, TR, BR, BL in marker frame.
        object_points = np.ascontiguousarray(np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64))

        # Empty / mis-sized distortion arrays segfault solvePnP on some builds.
        if self.dist_coeffs is None or self.dist_coeffs.size == 0:
            dist = np.zeros(5, dtype=np.float64)
        else:
            dist = np.ascontiguousarray(self.dist_coeffs.astype(np.float64))

        results = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            image_points = np.ascontiguousarray(
                marker_corners.reshape(-1, 2).astype(np.float64))
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    object_points, image_points,
                    self.camera_matrix, dist,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            except cv2.error as exc:
                self.get_logger().warn(f'solvePnP failed: {exc}')
                continue
            if not ok:
                continue
            x, y, z = float(tvec[0]), float(tvec[1]), float(tvec[2])
            distance = float(np.linalg.norm(tvec))
            bearing = float(np.arctan2(x, z))   # +ve = marker is to the right of optical axis
            results.append(Detection(
                marker_id=int(marker_id),
                distance_m=distance,
                bearing_rad=bearing,
                pose_camera=(x, y, z),
                corners_px=image_points.tolist(),
            ))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
