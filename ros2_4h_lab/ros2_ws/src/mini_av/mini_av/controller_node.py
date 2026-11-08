"""controller: 판단(FSM) + 조향 제어(PD) → /cmd_vel

구독  /lane_error         std_msgs/Float32
      /obstacle_distance  std_msgs/Float32  (obstacle 노드를 안 띄우면 inf 로 간주)
발행  /cmd_vel            geometry_msgs/Twist
      /drive_state        std_msgs/String   현재 FSM 상태
파라미터 speed, kp, kd, max_w, use_solution  (speed/kp/kd 는 실행 중에 ros2 param set 가능)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String

from mini_av import _algo


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.declare_parameter("use_solution", False)
        self.declare_parameter("speed", 0.35)
        self.declare_parameter("kp", 5.0)
        self.declare_parameter("kd", 0.3)
        self.declare_parameter("max_w", 2.5)
        self.algo = _algo.load(self.get_parameter("use_solution").value)

        self.lane_error = float("nan")
        self.obstacle = float("inf")
        self.fsm = "FOLLOW"
        self.lost_time = 0.0
        self.w = 0.0
        self.ctrl_state = {}
        self.create_subscription(Float32, "/lane_error", self.on_lane, 10)
        self.create_subscription(Float32, "/obstacle_distance", self.on_obs, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/drive_state", 10)
        self.dt = 1 / 30
        self.create_timer(self.dt, self.loop)

    def on_lane(self, msg):
        self.lane_error = msg.data

    def on_obs(self, msg):
        self.obstacle = msg.data

    def loop(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        err = self.lane_error
        self.lost_time = self.lost_time + self.dt if math.isnan(err) else 0.0

        new_state, scale = self.algo.decide(self.fsm, err, self.obstacle, p("speed"),
                                            self.lost_time, self.dt)
        if new_state != self.fsm:
            self.get_logger().info("상태 %s → %s (obstacle=%.2f m)"
                                   % (self.fsm, new_state, self.obstacle))
            self.fsm = new_state

        if not math.isnan(err):   # 차선을 놓치면 직전 조향 유지
            self.w = self.algo.steering_control(err, self.dt, self.ctrl_state,
                                                p("kp"), p("kd"), p("max_w"))
        cmd = Twist()
        cmd.linear.x = p("speed") * scale
        cmd.angular.z = self.w if scale > 0 else 0.0
        self.pub.publish(cmd)
        self.pub_state.publish(String(data=self.fsm))


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
