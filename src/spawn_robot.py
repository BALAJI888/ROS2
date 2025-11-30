#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        
        # Create client for spawn service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
            
        self.spawn_robot()

    def spawn_robot(self):
        """Spawn robot in Gazebo"""
        # Get URDF file path
        urdf_file = os.path.join(
            os.path.dirname(__file__),
            '..', 'urdf', 'diff_robot.urdf'
        )
        
        with open(urdf_file, 'r') as file:
            robot_urdf = file.read()
        
        # Create spawn request
        request = SpawnEntity.Request()
        request.name = 'docking_robot'
        request.xml = robot_urdf
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.1
        request.initial_pose.orientation.w = 1.0
        
        # Send request
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot spawned successfully!')
            else:
                self.get_logger().error(f'Failed to spawn robot: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main():
    rclpy.init()
    node = RobotSpawner()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
