from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='circle',
        description='Target motion mode: circle, sinusoidal, random_walk'
    )

    target_node = Node(
        package='uav_target_sim',
        executable='uav_target_sim',
        name='uav_target_sim',
        output='screen',
        parameters=[{
            'motion_mode': LaunchConfiguration('motion_mode'),
        }],
    )

    return LaunchDescription([mode_arg, target_node])
