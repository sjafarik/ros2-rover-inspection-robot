#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64
from sensor_msgs.msg import Image, JointState

from rover_interfaces.srv import CaptureCameraAtAngle


class CameraCaptureServiceServer(Node):

    def __init__(self):
        super().__init__('camera_capture_service_server')

        self.cb_group = ReentrantCallbackGroup()

        self.camera_joint_name = 'holder_camera_joint'
        self.angle_tolerance = 0.03
        self.angle_timeout_sec = 20.0
        self.image_timeout_sec = 20.0

        self.camera_cmd_pub = self.create_publisher(
            Float64,
            '/camera_pos_cmd',
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_group
        )

        self.srv = self.create_service(
            CaptureCameraAtAngle,
            'turn_camera_capture',
            self.handle_turn_camera_capture,
            callback_group=self.cb_group
        )

        self.latest_image = None
        self.latest_image_stamp_ns = None
        self.current_camera_angle = None

        self.lock = threading.Lock()
        self.image_event = threading.Event()
        self.joint_event = threading.Event()

        self.get_logger().info('Camera capture service server is ready.')

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            )
        self.image_event.set()

    def joint_state_callback(self, msg):
        try:
            idx = msg.name.index(self.camera_joint_name)
        except ValueError:
            return

        if idx >= len(msg.position):
            return

        with self.lock:
            self.current_camera_angle = msg.position[idx]

        self.joint_event.set()

    def angle_close_enough(self, target_angle):
        with self.lock:
            if self.current_camera_angle is None:
                return False
            current = self.current_camera_angle

        return abs(current - target_angle) <= self.angle_tolerance

    def wait_until_angle_reached(self, target_angle):
        start_time = self.get_clock().now()

        while rclpy.ok():
            if self.angle_close_enough(target_angle):
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.angle_timeout_sec:
                return False

            self.joint_event.wait(timeout=0.05)
            self.joint_event.clear()

        return False

    def wait_for_next_image_after(self, old_stamp):
        start_time = self.get_clock().now()

        while rclpy.ok():
            with self.lock:
                if self.latest_image is not None:
                    new_stamp = self.latest_image_stamp_ns
                    if old_stamp is None or (new_stamp is not None and new_stamp > old_stamp):
                        return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.image_timeout_sec:
                return False

            self.image_event.wait(timeout=0.05)
            self.image_event.clear()

        return False

    def handle_turn_camera_capture(self, request, response):
        target_angle = request.angle
        self.get_logger().info(f'Received request to turn camera to angle: {target_angle:.3f}')

        cmd_msg = Float64()
        cmd_msg.data = target_angle
        self.camera_cmd_pub.publish(cmd_msg)

        reached = self.wait_until_angle_reached(target_angle)

        if not reached:
            with self.lock:
                current = self.current_camera_angle
            response.success = False
            response.message = (
                f'Timed out waiting for camera joint to reach target. '
                f'Target={target_angle:.3f}, current={current}'
            )
            self.get_logger().warn(response.message)
            return response

        with self.lock:
            old_stamp = self.latest_image_stamp_ns

        got_image = self.wait_for_next_image_after(old_stamp)

        if not got_image:
            response.success = False
            response.message = 'Camera reached target angle, but timed out waiting for a fresh image.'
            self.get_logger().warn(response.message)
            return response

        with self.lock:
            response.image = self.latest_image

        response.success = True
        response.message = f'Camera reached {target_angle:.3f} rad and image captured.'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    node = CameraCaptureServiceServer()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
