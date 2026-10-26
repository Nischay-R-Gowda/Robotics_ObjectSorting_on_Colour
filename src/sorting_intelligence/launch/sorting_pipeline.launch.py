from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sorting_intelligence',
            executable='perception_node',
            name='perception_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='sorting_intelligence',
            executable='task_manager',
            name='task_manager',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
