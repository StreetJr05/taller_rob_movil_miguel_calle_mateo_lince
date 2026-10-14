import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg = get_package_share_directory('pkg_rob_dif')
    xacro_path = os.path.join(pkg, 'urdf', 'rob_dif.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'pista.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'rob_dif.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', world_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'robot_diferencial',
                       '-x', '1.0', '-y', '0.0', '-z', '0.02', '-Y', '0.0']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config,
                         'use_sim_time': True}]
        ),
        Node(
            package='pkg_rob_dif',
            executable='cinematica_llantas.py',
            output='screen',
            # Must match wheel_radius and wheel_separation in the URDF
            parameters=[{'wheel_radius': 0.05,
                         'wheel_separation': 0.36,
                         'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
