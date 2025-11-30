#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot driver node started')

    def control_loop(self):
        """Simple control loop for basic movement"""
        # This can be used for manual control or testing
        pass

    def move_forward(self, speed=0.2):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angular_speed=0.5):
        """Rotate the robot"""
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = RobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
