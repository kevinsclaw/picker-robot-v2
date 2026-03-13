"""
Picker V2 Gazebo 仿真 Launch 文件
=================================
启动 Gazebo + 加载 Picker V2 URDF + 控制器
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'picker_v2.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'pick_place.sdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 启动 Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'picker_v2',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.0',
            ],
            output='screen',
        ),

        # Bridge: Gazebo ↔ ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/joint_commands@std_msgs/msg/Float64MultiArray]gz.msgs.Double_V',
            ],
            output='screen',
        ),
    ])
