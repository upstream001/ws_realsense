#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to RGB image topic
        self.rgb_subscription = self.create_subscription(
            Image,
            'rgb_image',
            self.rgb_callback,
            10)

        # Subscribe to depth image topic
        self.depth_subscription = self.create_subscription(
            Image,
            'depth_image',
            self.depth_callback,
            10)

        # Create windows for displaying images
        cv2.namedWindow('RGB Image', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Depth Image', cv2.WINDOW_AUTOSIZE)

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Convert RGB to BGR for OpenCV display
            cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # Display the image
            cv2.imshow('RGB Image', cv_image_bgr)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            # Normalize depth image for visualization
            cv_image_normalized = cv2.normalize(
                cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

            # Apply colormap for better visualization
            cv_image_colormap = cv2.applyColorMap(
                cv_image_normalized, cv2.COLORMAP_JET)

            # Display the image
            cv2.imshow('Depth Image', cv_image_colormap)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    camera_viewer = CameraViewer()

    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        camera_viewer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
