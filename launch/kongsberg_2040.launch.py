import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('kongsberg_em_driver'), 'launch', 'kongsberg_2040.yaml')

    return LaunchDescription([
        Node(
            package='kongsberg_em_driver',
            executable='kongsberg_em_node',
            name='kongsberg_2040',
            output='screen',
            parameters=[params],
        ),

        # Kongsberg vessel coordinate system has X forward, Y to starboard, Z down so to come
        # back to the standard ROS coordinate system (Z up,
        # https://www.ros.org/reps/rep-0103.html) we need to roll this by 180 degrees.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_sonar_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '3.1415926', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'ship_ref_point', '--child-frame-id', 'sonar_vcs',
            ],
        ),
    ])
