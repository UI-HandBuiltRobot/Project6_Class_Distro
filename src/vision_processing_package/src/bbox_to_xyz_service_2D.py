#!/usr/bin/env python3

import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_interfaces.srv import BboxToXYZ


# Robot datum offsets relative to the calibration datum in millimeters.
X_OFFSET_MM = 105.0
Y_OFFSET_MM = -90.0


class BboxToXYZServiceNode(Node):
    def __init__(self):
        """Register the endpoint and load camera calibration for 2D point mapping."""
        super().__init__('bbox_to_xyz_node')

        pkg_share = get_package_share_directory('vision_processing_package')
        calibration_path = os.path.join(pkg_share, 'config', 'camera_calibration.json')
        if not os.path.exists(calibration_path):
            calibration_path = os.path.join(pkg_share, 'camera_calibration.json')
        self._load_calibration(calibration_path)

        self.create_service(BboxToXYZ, 'bbox_to_xyz_service', self._handle_request)

        self._bridge = CvBridge()
        self._img_pub = self.create_publisher(Image, 'img_corrected', 10)
        self.create_subscription(Image, 'img_raw', self._img_raw_callback, 10)

        self.get_logger().info('BboxToXYZ Service ready (2D calibrated mapping).')

    def _load_calibration(self, calibration_path):
        """Load intrinsics, distortion, perspective transform, and scaling from JSON."""
        with open(calibration_path, 'r', encoding='utf-8') as file:
            calibration = json.load(file)

        intrinsics = calibration.get('intrinsics', {})
        extrinsics = calibration.get('extrinsics', {})
        scaling = calibration.get('scaling', {})
        calibration_info = calibration.get('calibration_info', {})

        image_size = calibration_info.get('image_size', [1920, 1080])
        if len(image_size) == 2 and image_size[0] > 0 and image_size[1] > 0:
            self.default_image_width = float(image_size[0])
            self.default_image_height = float(image_size[1])
        else:
            self.default_image_width = 1920.0
            self.default_image_height = 1080.0

        self.camera_matrix = np.asarray(
            intrinsics.get('camera_matrix', []), dtype=np.float64
        )
        distortion = np.asarray(
            intrinsics.get('distortion_coefficients', []), dtype=np.float64
        )
        if distortion.ndim == 2 and distortion.shape[0] == 1:
            distortion = distortion[0]
        self.distortion_coeffs = distortion.reshape(-1, 1)

        self.perspective_matrix = np.asarray(
            extrinsics.get('perspective_matrix', []), dtype=np.float64
        )

        pixels_per_real_unit = scaling.get('pixels_per_real_unit', 0.0)
        if pixels_per_real_unit and pixels_per_real_unit > 0.0:
            self.pixels_per_mm = float(pixels_per_real_unit)
        else:
            square_size_pixels = float(scaling.get('square_size_pixels', 0.0))
            square_size_real = float(scaling.get('square_size_real', 0.0))
            if square_size_real <= 0.0 or square_size_pixels <= 0.0:
                raise ValueError('Invalid scaling in camera_calibration.json')
            self.pixels_per_mm = square_size_pixels / square_size_real

        if self.camera_matrix.shape != (3, 3):
            raise ValueError('camera_matrix must be 3x3 in camera_calibration.json')
        if self.perspective_matrix.shape != (3, 3):
            raise ValueError('perspective_matrix must be 3x3 in camera_calibration.json')
        if self.pixels_per_mm <= 0.0:
            raise ValueError('pixels_per_mm must be positive in camera_calibration.json')

    def _img_raw_callback(self, msg: Image):
        """Undistort and perspective-correct an incoming raw image, then republish."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        h, w = frame.shape[:2]
        undistorted = cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs)
        corrected = cv2.warpPerspective(undistorted, self.perspective_matrix, (w, h))

        out_msg = self._bridge.cv2_to_imgmsg(corrected, encoding='bgr8')
        out_msg.header = msg.header
        self._img_pub.publish(out_msg)

    def _handle_request(self, request, response):
        """Map bbox center pixel -> undistort -> perspective-correct -> millimeter coordinates."""
        bbox_x = float(request.bbox_x)
        bbox_y = float(request.bbox_y)

        if request.image_width > 0 and request.image_height > 0:
            image_width = float(request.image_width)
            image_height = float(request.image_height)

            # Handle callers that still send normalized coordinates with explicit image dimensions.
            if 0.0 <= bbox_x <= 1.0 and 0.0 <= bbox_y <= 1.0:
                pixel_x = bbox_x * image_width
                pixel_y = bbox_y * image_height
            else:
                pixel_x = bbox_x
                pixel_y = bbox_y
        else:
            # If dimensions are omitted, treat request coordinates as normalized [0, 1].
            pixel_x = bbox_x * self.default_image_width
            pixel_y = bbox_y * self.default_image_height

        input_points = np.array([[[pixel_x, pixel_y]]], dtype=np.float64)

        if pixel_x < 0.0 or pixel_y < 0.0:
            self.get_logger().error(
                f'Invalid bbox center pixel ({pixel_x:.3f}, {pixel_y:.3f}); cannot compute 2D position.'
            )
            response.success = False
            response.x_mm = 0.0
            response.y_mm = 0.0
            response.z_mm = 0.0
            return response

        undistorted = cv2.undistortPoints(
            src=input_points,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.distortion_coeffs,
            P=self.camera_matrix,
        )
        corrected = cv2.perspectiveTransform(undistorted, self.perspective_matrix)

        corrected_x_px = float(corrected[0, 0, 0])
        corrected_y_px = float(corrected[0, 0, 1])

        # Convert corrected pixel coordinates into calibration-frame millimeters.
        x_mm = corrected_x_px / self.pixels_per_mm
        y_mm = corrected_y_px / self.pixels_per_mm

        # Rotate reported axes so calibration Y becomes robot X, and calibration X becomes robot Y.
        rotated_x_mm = y_mm
        rotated_y_mm = x_mm

        # Shift from calibration datum into robot datum coordinates.
        final_x_mm = rotated_x_mm + X_OFFSET_MM
        final_y_mm = rotated_y_mm + Y_OFFSET_MM

        self.get_logger().info(
            'BboxToXYZ 2D: '
            f'pixel=({pixel_x:.1f}, {pixel_y:.1f}) -> '
            f'undist/persp px=({corrected_x_px:.1f}, {corrected_y_px:.1f}) -> '
            f'cal_mm=({x_mm:.1f}, {y_mm:.1f}) -> '
            f'robot_mm=({final_x_mm:.1f}, {final_y_mm:.1f})'
        )

        response.success = True
        response.x_mm = float(final_x_mm)
        response.y_mm = float(final_y_mm)
        response.z_mm = 0.0

        return response


def main(args=None):
    """Run the BboxToXYZ node until interrupted, then shut ROS down cleanly."""
    rclpy.init(args=args)
    node = BboxToXYZServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()