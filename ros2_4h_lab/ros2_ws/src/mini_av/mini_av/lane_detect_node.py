"""lane_detect: 카메라 영상 → 차선 중심 오프셋

구독  /camera/image_raw  sensor_msgs/Image (mono8)
발행  /lane_error        std_msgs/Float32  차선 중심 좌우 오프셋 [m], 왼쪽 +. 못 찾으면 NaN
      /lane/debug        sensor_msgs/Image 인식 결과 확인용 (흰 세로줄 = 찾은 차선 중심)
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from mini_av import _algo, core


class LaneDetect(Node):
    def __init__(self):
        super().__init__("lane_detect")
        self.declare_parameter("use_solution", False)
        self.declare_parameter("lane_width", 0.35)
        self.algo = _algo.load(self.get_parameter("use_solution").value)
        self.cam = core.BevCamera()          # 열 ↔ 미터 변환용 (시뮬레이터와 같은 설정)
        self.state = {}
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.pub = self.create_publisher(Float32, "/lane_error", 10)
        self.pub_dbg = self.create_publisher(Image, "/lane/debug", 1)
        self.get_logger().info("lane_detect 시작 (%s)" % self.algo.__name__)

    # 15hz마다 이미지 올때 콜백으로 호출됨. 
    # msg.data는 바이트 덩어리라 reshape으로 120 * 160 numpy배열로 바꿈. 
    def on_image(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        t0 = time.perf_counter()
        err = self.algo.find_lane_center(img, self.cam, self.get_parameter("lane_width").value,
                                         self.state)
        ms = (time.perf_counter() - t0) * 1000
        self.pub.publish(Float32(data=float(err)))
        self.get_logger().info("error=%+.3f m  처리 %.1f ms" % (err, ms),
                               throttle_duration_sec=2.0)

        dbg = img.copy()
        if not math.isnan(err):
            col = int(round((self.cam.half_width - err) / (2 * self.cam.half_width)
                            * (self.cam.width - 1)))
            if 0 <= col < dbg.shape[1]:
                dbg[:, max(col - 1, 0):col + 2] = 255
        dbg[:, dbg.shape[1] // 2] = 128   # 회색 = 차량 중심
        out = Image(header=msg.header, height=dbg.shape[0], width=dbg.shape[1],
                    encoding="mono8", step=dbg.shape[1], data=dbg.tobytes())
        self.pub_dbg.publish(out)


def main():
    rclpy.init()
    node = LaneDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
