#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from std_msgs.msg import Bool
import numpy as np
import math

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Parameters
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('docking_distance', 0.1)
        self.declare_parameter('alignment_threshold', 0.05)
        self.declare_parameter('linear_threshold', 0.02)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.docking_distance = self.get_parameter('docking_distance').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value
        self.linear_threshold = self.get_parameter('linear_threshold').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # State variables
        self.target_pose = None
        self.is_docked = False
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.docking_status_pub = self.create_publisher(Bool, '/docking_status', 10)
        self.detection_sub = self.create_subscription(
            PoseArray,
            '/aruco_detections',
            self.detection_callback,
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('Docking controller node started')
        self.get_logger().info(f'Targeting marker ID: {self.target_marker_id}')

    def detection_callback(self, msg):
        """Process detected markers"""
        # For simplicity, we'll assume the first detected marker is our target
        # In a real scenario, you'd filter by marker ID
        if len(msg.poses) > 0:
            self.target_pose = msg.poses[0]  # Use first detected marker
            self.get_logger().debug(f'Target pose: {self.target_pose.position}')

    def control_loop(self):
        """Main control loop for docking"""
        if self.target_pose is None or self.is_docked:
            return
            
        # Extract position
        x = self.target_pose.position.x
        y = self.target_pose.position.y
        z = self.target_pose.position.z
        
        # Calculate distance to marker
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        # Calculate angle to marker
        angle_to_marker = math.atan2(y, x)
        
        # Create control message
        cmd_vel = Twist()
        
        # Check if docking is complete
        if distance <= self.docking_distance and abs(angle_to_marker) < self.alignment_threshold:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.is_docked = True
            self.get_logger().info('Docking completed successfully!')
        else:
            # Linear control (approach marker)
            linear_error = distance - self.docking_distance
            cmd_vel.linear.x = self.linear_kp * linear_error
            cmd_vel.linear.x = max(min(cmd_vel.linear.x, self.max_linear_speed), -self.max_linear_speed)
            
            # Angular control (align with marker)
            cmd_vel.angular.z = -self.angular_kp * angle_to_marker
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
            
            self.get_logger().debug(f'Distance: {distance:.3f}, Angle: {angle_to_marker:.3f}')
            self.get_logger().debug(f'Control: linear={cmd_vel.linear.x:.3f}, angular={cmd_vel.angular.z:.3f}')
        
        # Publish control command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Publish docking status
        status_msg = Bool()
        status_msg.data = self.is_docked
        self.docking_status_pub.publish(status_msg)

def main():
    rclpy.init()
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot when shutting down
        cmd_vel = Twist()
        node.cmd_vel_pub.publish(cmd_vel)
        node.get_logger().info('Shutting down, robot stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
