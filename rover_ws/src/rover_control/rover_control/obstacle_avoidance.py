import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.safe_distance = 0.8

    def scan_callback(self, msg):

        ranges = msg.ranges
        center_index = len(ranges) // 2

        window = ranges[center_index-20:center_index+20]

        min_distance = min(window)

        cmd = Twist()

        if min_distance < self.safe_distance:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

            self.get_logger().info("Obstacle detected → turning")

        else:

            cmd.linear.x = 0.4
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()