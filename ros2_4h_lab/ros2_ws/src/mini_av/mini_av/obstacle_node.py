"""obstacle: LiDAR → 전방 최소 거리

구독  /scan               sensor_msgs/LaserScan
발행  /obstacle_distance  std_msgs/Float32  전방 ±half_fov_deg 안 최소 거리 [m]. 없으면 inf
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from mini_av import _algo


class Obstacle(Node):
    def __init__(self):
        super().__init__("obstacle")
        self.declare_parameter("use_solution", False)
        self.declare_parameter("half_fov_deg", 20.0)
        self.algo = _algo.load(self.get_parameter("use_solution").value)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.pub = self.create_publisher(Float32, "/obstacle_distance", 10)

    def on_scan(self, msg):
        d = self.algo.front_distance(msg.ranges, msg.angle_min, msg.angle_increment,
                                     self.get_parameter("half_fov_deg").value)
        self.pub.publish(Float32(data=float(d)))


def main():
    rclpy.init()
    node = Obstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
