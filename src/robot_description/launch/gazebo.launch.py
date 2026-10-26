import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'worlds',
        'sorting_world.sdf'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'sorting_arm.urdf.xacro'
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'config',
        'controllers.yaml'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                xacro_file
            ]),
            value_type=str
        )
    }

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'sorting_world',
            '-name', 'sorting_arm',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controller_config
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,

        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=8.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=10.0,
            actions=[arm_controller_spawner]
        ),

        TimerAction(
            period=12.0,
            actions=[gripper_controller_spawner]
        )
    ])
