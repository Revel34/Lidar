from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('rover30_slam'),
        'config',
        'slam_rviz.rviz'
    )

    return LaunchDescription([
        # 1️⃣ Uruchom odometry node z use_sim_time=True
        Node(
            package='rover30_slam',      # Twoja paczka z odometry
            executable='odom_from_joint_states',
            name='odom_from_joint_states',
            output='screen',
            parameters=[{"use_sim_time": True}],
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        # 2️⃣ Uruchom SLAM Toolbox (czas symulowany)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rover30_slam', 'slam.launch.py'],
            output='screen',
        ),

        # 3️⃣ Uruchom RViz2 z use_sim_time=True
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{"use_sim_time": True}]
        ),
    ])