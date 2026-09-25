"""
vision_png.launch.py

启动 uav_vision_png 节点，加载 config/params.yaml 参数文件。

用法：
  ros2 launch uav_vision_png vision_png.launch.py
  ros2 launch uav_vision_png vision_png.launch.py speed_cmd:=8.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_vision_png')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 允许命令行覆盖关键参数
    declare_speed = DeclareLaunchArgument(
        'speed_cmd', default_value='5.0',
        description='期望冲击速度上限 (m/s)')

    declare_N = DeclareLaunchArgument(
        'N_png', default_value='4.0',
        description='比例导引系数（导航增益）')

    declare_csv = DeclareLaunchArgument(
        'csv_path',
        default_value='/home/verser/ros2_ws/vpng_intercept_stats.csv',
        description='拦截统计 CSV 输出路径')

    vision_png_node = Node(
        package='uav_vision_png',
        executable='uav_vision_png',
        name='uav_vision_png',
        output='screen',
        parameters=[
            params_file,
            {
                # 命令行参数可覆盖 yaml 中同名参数
                'speed_cmd': LaunchConfiguration('speed_cmd'),
                'N_png':     LaunchConfiguration('N_png'),
                'csv_path':  LaunchConfiguration('csv_path'),
            }
        ],
        # 打印节点完整输出（方便调试）
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_speed,
        declare_N,
        declare_csv,
        vision_png_node,
    ])
