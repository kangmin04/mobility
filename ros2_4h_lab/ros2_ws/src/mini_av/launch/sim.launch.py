"""시뮬레이터만 띄운다 (teleop 으로 직접 몰아볼 때)"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="mini_av", executable="sim", name="mini_sim", output="screen"),
    ])
