from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch arguments
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution([
            FindPackageShare('onrobot_vgc10_description'),
            'urdf',
            'onrobot_vgc10_4cups.urdf.xacro'
        ]),
        description='Absolute path to URDF (xacro) file'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare('onrobot_vgc10_description'),
            'rviz',
            'urdf.rviz'
        ]),
        description='RViz config file'
    )

    declare_gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Use joint_state_publisher_gui'
    )

    # Load robot_description from xacro
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    return LaunchDescription([
        declare_model_arg,
        declare_rviz_arg,
        declare_gui_arg,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
            remappings=[('/joint_states', '/onrobot_sd/joint_states')]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            remappings=[('/joint_states', '/onrobot_sd/joint_states')]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        ),
    ])