#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import cv2

from rover_interfaces.srv import CaptureCameraAtAngle


class CameraCaptureServiceClient(Node):

    def __init__(self):
        super().__init__('camera_capture_service_client')
        self.client = self.create_client(CaptureCameraAtAngle, 'turn_camera_capture')
        self.bridge = CvBridge()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service turn_camera_capture...')

    def send_request(self, angle):
        request = CaptureCameraAtAngle.Request()
        request.angle = angle
        return self.client.call_async(request)

    def display_image(self, ros_image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                ros_image_msg,
                desired_encoding='bgr8'
            )

            cv2.imshow('Captured Camera Image', cv_image)
            cv2.waitKey(0)   # waits until a key is pressed
            cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f'Failed to display image: {e}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run rover_control camera_capture_service_client.py <angle>')
        return

    angle = float(sys.argv[1])

    node = CameraCaptureServiceClient()
    future = node.send_request(angle)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'success: {response.success}')
        node.get_logger().info(f'message: {response.message}')
        node.get_logger().info(
            f'image size: {response.image.width}x{response.image.height}, encoding={response.image.encoding}'
        )

        if response.success:
            node.display_image(response.image)
    else:
        node.get_logger().error('Service call failed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()