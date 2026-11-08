from glob import glob

from setuptools import setup

package_name = "mini_av"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kangmin",
    description="4시간 ROS 2 자율주행 실습",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim = mini_av.sim_node:main",
            "lane_detect = mini_av.lane_detect_node:main",
            "obstacle = mini_av.obstacle_node:main",
            "controller = mini_av.controller_node:main",
            "turtle_goal = mini_av.turtle_goal:main",
            "turtle_goal_solution = mini_av.turtle_goal_solution:main",
        ],
    },
)
