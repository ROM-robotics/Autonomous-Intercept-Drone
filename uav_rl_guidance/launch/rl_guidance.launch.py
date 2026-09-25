"""启动学习制导节点（加载 config/params.yaml）

用法:
  ros2 launch uav_rl_guidance rl_guidance.launch.py
  ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true  # PNG 基线
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_rl_guidance")
    params = os.path.join(pkg_share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("fallback_png", default_value="false",
                              description="true = 全程 PNG 基线（A/B 对比）"),
        DeclareLaunchArgument("bench_test", default_value="false",
                              description="true = 跳过起飞，直接 SEARCHING（台架调试）"),
        DeclareLaunchArgument("csv_path",
                              default_value="/home/verser/ros2_ws/rl_intercept_stats.csv"),
        Node(
            package="uav_rl_guidance",
            executable="uav_rl_guidance",
            name="uav_rl_guidance",
            output="screen",
            parameters=[
                params,
                {"fallback_png": LaunchConfiguration("fallback_png"),
                 "bench_test": LaunchConfiguration("bench_test"),
                 "csv_path": LaunchConfiguration("csv_path")},
            ],
        ),
    ])
