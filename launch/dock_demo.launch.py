import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Include Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('autonomous_docking'), 'launch', 'gazebo_world.launch.py')
        ])
    )
    
    # ArUco detector node
    aruco_detector = Node(
        package='autonomous_docking',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('autonomous_docking'), 'config', 'aruco_params.yaml')]
    )
    
    # Docking controller node
    docking_controller = Node(
        package='autonomous_docking',
        executable='docking_controller.py',
        name='docking_controller',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('autonomous_docking'), 'config', 'aruco_params.yaml')]
    )
    
    return LaunchDescription([
        gazebo_launch,
        aruco_detector,
        docking_controller,
    ])
