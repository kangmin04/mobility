"""시뮬레이터 + 차선 인식 + (선택)장애물 + 제어 를 한 번에 띄운다.

  ros2 launch mini_av lane_follow.launch.py                       # 내 코드로
  ros2 launch mini_av lane_follow.launch.py use_solution:=true    # 정답으로
  ros2 launch mini_av lane_follow.launch.py obstacles:=true       # 장애물 켜고
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("mini_av"), "config", "params.yaml")
    use_sol = ParameterValue(LaunchConfiguration("use_solution"), value_type=bool)
    obstacles = ParameterValue(LaunchConfiguration("obstacles"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_solution", default_value="false"),
        DeclareLaunchArgument("obstacles", default_value="false"),
        Node(package="mini_av", executable="sim", name="mini_sim", output="screen",
             parameters=[params, {"obstacles_enabled": obstacles}]),
        Node(package="mini_av", executable="lane_detect", name="lane_detect", output="screen",
             parameters=[params, {"use_solution": use_sol}]),
        Node(package="mini_av", executable="obstacle", name="obstacle", output="screen",
             parameters=[params, {"use_solution": use_sol}]),
        Node(package="mini_av", executable="controller", name="controller", output="screen",
             parameters=[params, {"use_solution": use_sol}]),
    ])
