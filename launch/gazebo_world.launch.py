import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    pkg_path = get_package_share_directory('autonomous_docking')
    
    # Gazebo world file
    world_file = os.path.join(pkg_path, 'worlds', 'docking_world.world')
    
    # Start Gazebo with the specified world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='autonomous_docking',
        executable='spawn_robot.py',
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[os.path.join(pkg_path, 'urdf', 'diff_robot.urdf')]
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
    ])
