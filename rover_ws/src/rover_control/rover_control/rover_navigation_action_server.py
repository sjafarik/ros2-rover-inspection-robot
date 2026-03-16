#! /usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rover_interfaces.action import NavigateToPoint

DISTANCE_THRESHOLD = 0.10


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NavigateActionServer(Node):
    def __init__(self):
        super().__init__("rover_navigation_action_server")

        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            "navigate",
            execute_callback=self.navigate_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.tf_sub = self.create_subscription(
            TFMessage,
            "/world_pose_tf",
            self.update_robot_pose,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.latest_scan = None

        # Reactive navigation mode
        self.mode = "GO_TO_GOAL"      # or BYPASS
        self.bypass_side = "left"     # or right
        self.goal_clear_count = 0

        # Tuning
        self.safe_distance = 3.0
        self.front_stop_distance = 2.0
        self.goal_clear_required_cycles = 15

        self.max_linear_speed = 1.0
        self.max_angular_speed = 2.0

        self.turn_in_place_gain = 2.0
        self.drive_turn_gain = 1.4

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal: x={goal_request.goal_point.x:.3f}, "
            f"y={goal_request.goal_point.y:.3f}, "
            f"z={goal_request.goal_point.z:.3f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def update_robot_pose(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "rover_robot":
                self.robot_x = transform.transform.translation.x
                self.robot_y = transform.transform.translation.y
                self.robot_yaw = yaw_from_quat(transform.transform.rotation)
                break

    def scan_callback(self, msg):
        self.latest_scan = msg

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_valid_range(self, index):
        if self.latest_scan is None:
            return float("inf")

        if index < 0 or index >= len(self.latest_scan.ranges):
            return float("inf")

        value = self.latest_scan.ranges[index]

        if math.isnan(value) or math.isinf(value):
            return float("inf")

        if value < self.latest_scan.range_min or value > self.latest_scan.range_max:
            return float("inf")

        return value

    def angle_to_index(self, angle_rad):
        scan = self.latest_scan
        if scan is None:
            return None

        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            return None

        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        index = max(0, min(index, len(scan.ranges) - 1))
        return index

    def index_to_angle(self, index):
        scan = self.latest_scan
        return scan.angle_min + index * scan.angle_increment

    def sector_min(self, start_deg, end_deg):
        if self.latest_scan is None:
            return float("inf")

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        start_idx = self.angle_to_index(start_rad)
        end_idx = self.angle_to_index(end_rad)

        if start_idx is None or end_idx is None:
            return float("inf")

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        values = []
        for i in range(start_idx, end_idx + 1):
            r = self.get_valid_range(i)
            if math.isfinite(r):
                values.append(r)

        if not values:
            return float("inf")

        return min(values)

    def front_blocked(self):
        return self.sector_min(-15.0, 15.0) < self.front_stop_distance

    def choose_bypass_side(self):
        left_min = self.sector_min(20.0, 80.0)
        right_min = self.sector_min(-80.0, -20.0)

        self.get_logger().info(
            f"choose_bypass_side: left_min={left_min:.2f}, right_min={right_min:.2f}"
        )

        if left_min >= right_min:
            return "left"
        return "right"

    def heading_is_free(self, heading_rad, clearance=None):
        """
        Check whether a heading direction is free in a small angular window.
        """
        if self.latest_scan is None:
            return False

        if clearance is None:
            clearance = self.safe_distance

        window_deg = 8.0
        center_deg = math.degrees(heading_rad)

        min_dist = self.sector_min(center_deg - window_deg, center_deg + window_deg)
        return min_dist > clearance

    def find_best_free_heading(self, target_heading_rad, restrict_side=None):
        """
        Find a safe heading closest to target_heading_rad.
        If restrict_side is 'left', only consider headings >= 0.
        If restrict_side is 'right', only consider headings <= 0.
        """
        scan = self.latest_scan
        if scan is None:
            return 0.0

        best_heading = None
        best_score = float("inf")

        for i in range(len(scan.ranges)):
            heading = self.index_to_angle(i)

            if restrict_side == "left" and heading < 0.0:
                continue
            if restrict_side == "right" and heading > 0.0:
                continue

            r = self.get_valid_range(i)
            if not math.isfinite(r):
                # Treat inf as open space
                r = scan.range_max

            if r < self.safe_distance:
                continue

            # score = closeness to goal heading
            score = abs(normalize_angle(heading - target_heading_rad))

            # prefer wider clearance a bit
            score -= 0.05 * min(r, 3.0)

            if score < best_score:
                best_score = score
                best_heading = heading

        if best_heading is None:
            # fallback: turn hard toward chosen side
            if restrict_side == "left":
                return math.radians(60.0)
            if restrict_side == "right":
                return math.radians(-60.0)
            return 0.0

        return best_heading

    def compute_local_heading(self, goal_heading_rad):
        """
        Core idea:
        - In GO_TO_GOAL, use goal heading if free, otherwise enter BYPASS.
        - In BYPASS, keep choosing safe headings on one chosen side.
        - Leave BYPASS only after goal heading stays free for several cycles.
        """
        if self.latest_scan is None:
            return goal_heading_rad

        goal_heading_clamped = max(
            self.latest_scan.angle_min,
            min(self.latest_scan.angle_max, goal_heading_rad)
        )

        goal_heading_free = self.heading_is_free(goal_heading_clamped)

        if self.mode == "GO_TO_GOAL":
            if goal_heading_free and not self.front_blocked():
                self.goal_clear_count = 0
                return goal_heading_clamped

            self.mode = "BYPASS"
            self.bypass_side = self.choose_bypass_side()
            self.goal_clear_count = 0

            self.get_logger().info(
                f"Entering BYPASS on {self.bypass_side} side"
            )

        # BYPASS mode
        if goal_heading_free and not self.front_blocked():
            self.goal_clear_count += 1
        else:
            self.goal_clear_count = 0

        if self.goal_clear_count >= self.goal_clear_required_cycles:
            self.mode = "GO_TO_GOAL"
            self.goal_clear_count = 0
            self.get_logger().info("Leaving BYPASS -> GO_TO_GOAL")
            return goal_heading_clamped

        # keep following a safe heading on the chosen side
        bypass_heading = self.find_best_free_heading(
            target_heading_rad=goal_heading_clamped,
            restrict_side=self.bypass_side
        )

        return bypass_heading

    def navigate_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        goal_x = goal_handle.request.goal_point.x
        goal_y = goal_handle.request.goal_point.y

        start_time = self.get_clock().now()

        self.mode = "GO_TO_GOAL"
        self.bypass_side = "left"
        self.goal_clear_count = 0

        while rclpy.ok() and (
            self.robot_x is None or
            self.robot_y is None or
            self.robot_yaw is None or
            self.latest_scan is None
        ):
            self.get_logger().info("Waiting for robot pose and lidar scan...")
            rclpy.spin_once(self, timeout_sec=0.1)

        feedback_msg = NavigateToPoint.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")

                result = NavigateToPoint.Result()
                elapsed = self.get_clock().now() - start_time
                result.elapsed_time = elapsed.nanoseconds / 1e9
                return result

            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance_to_point = math.hypot(dx, dy)

            feedback_msg.distance_to_point = float(distance_to_point)
            goal_handle.publish_feedback(feedback_msg)

            if distance_to_point <= DISTANCE_THRESHOLD:
                break

            goal_heading_world = math.atan2(dy, dx)
            goal_heading_robot = normalize_angle(goal_heading_world - self.robot_yaw)

            local_heading = self.compute_local_heading(goal_heading_robot)
            heading_error = normalize_angle(local_heading)

            cmd = Twist()

            # Emergency stop-turn if something is too close straight ahead
            if self.front_blocked():
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6 if self.bypass_side == "left" else -0.6
            else:
                if abs(heading_error) > 0.35:
                    cmd.linear.x = 0.0
                    cmd.angular.z = max(
                        -self.max_angular_speed,
                        min(self.max_angular_speed,
                            self.turn_in_place_gain * heading_error)
                    )
                else:
                    front_min = self.sector_min(-15.0, 15.0)
                    clearance_scale = min(1.0, max(0.3, front_min / 2.0))

                    cmd.linear.x = min(
                        self.max_linear_speed * clearance_scale,
                        0.5 * distance_to_point
                    )
                    cmd.angular.z = max(
                        -0.6,
                        min(0.6, self.drive_turn_gain * heading_error)
                    )

            self.get_logger().info(
                f"mode={self.mode}, side={self.bypass_side}, "
                f"goal_heading={math.degrees(goal_heading_robot):.1f} deg, "
                f"local_heading={math.degrees(local_heading):.1f} deg, "
                f"front_min={self.sector_min(-15.0, 15.0):.2f}, "
                f"dist={distance_to_point:.2f}"
            )

            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()
        goal_handle.succeed()

        result = NavigateToPoint.Result()
        elapsed = self.get_clock().now() - start_time
        result.elapsed_time = elapsed.nanoseconds / 1e9

        self.get_logger().info(
            f"Goal reached in {result.elapsed_time:.2f} seconds"
        )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServer()
    node.get_logger().info("Action Server Running...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Terminating Node...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()