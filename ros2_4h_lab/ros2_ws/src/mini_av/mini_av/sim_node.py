"""mini_sim: 2D 자율주행 시뮬레이터 노드 (Gazebo 없이 가볍게 돈다).

구독  /cmd_vel              geometry_msgs/Twist   linear.x 속도, angular.z 요레이트
발행  /camera/image_raw     sensor_msgs/Image     mono8 BEV 영상 (15 Hz)
      /scan                 sensor_msgs/LaserScan 360빔 (10 Hz)
      /odom                 nav_msgs/Odometry     + TF odom→base_link
      /sim/topdown          sensor_msgs/Image     rgb8 전체 지도 (5 Hz)
      /sim/status           std_msgs/String       JSON: 바퀴 수, 이탈 횟수, 충돌 ...
서비스 /sim/reset            std_srvs/Empty        처음 위치로
파라미터 obstacles_enabled (bool, 실행 중 변경 가능)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from mini_av import core


# 구조 요약: 이 노드는 "ROS 포장지" 역할만 함. 실제 계산(차 운동, 카메라 그림, LiDAR, 채점)은
# 전부 core.py 객체가 하고, 여기서는 그 결과를 주기별 타이머로 꺼내 메시지로 포장해 발행함.
# 입력은 /cmd_vel 하나뿐 → 콜백은 값만 저장하고, 60 Hz _physics 타이머가 그 값을 꺼내 씀.
class MiniSim(Node):
    def __init__(self):
        super().__init__("mini_sim")
        # 1. 파라미터 선언 (ros2 param set /mini_sim ... 로 실행 중 변경 가능)
        self.declare_parameter("obstacles_enabled", False)
        self.declare_parameter("camera_noise", 8.0)
        self.declare_parameter("publish_topdown", True)
        self.declare_parameter("cmd_timeout", 0.5)
        self.add_on_set_parameters_callback(self._on_params)   # obstacles_enabled 변경을 즉시 반영하려고 등록

        # 2. 시뮬레이션 "세상" 객체 생성 (전부 ROS 의존성 없는 core.py 클래스)
        self.track = core.StadiumTrack()
        self.car = core.Car()
        self.cam = core.BevCamera()
        self.lidar = core.Lidar()
        self.ref = core.Referee(self.track)
        self.view = core.TopDownView(self.track)
        self.rng = np.random.default_rng()
        self.box_list = [(0.0, self.track.R, 0.2)]   # 위쪽 직선 가운데 박스
        self.obstacles = self.get_parameter("obstacles_enabled").value
        self.trail = []          # topdown 에 그릴 파란 주행 궤적 점들
        # 3. 차를 출발 위치에 놓고 채점 초기화
        self._reset()

        # 4. 통신 연결: 구독 1개, 발행 5개 + TF, 서비스 1개
        self.cmd = (0.0, 0.0)                         # 가장 최근 (v, ω) 명령. 처음엔 정지
        self.last_cmd_time = self.get_clock().now()   # 명령 끊김(timeout) 판단용
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.pub_img = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_top = self.create_publisher(Image, "/sim/topdown", 1)   # 큐 1: 보기용이라 최신 한 장만 있으면 됨
        self.pub_status = self.create_publisher(String, "/sim/status", 10)
        self.tf = TransformBroadcaster(self)
        self.create_service(Empty, "/sim/reset", self._on_reset)

        # 5. 타이머 5개: 물리는 촘촘하게(60 Hz), 센서는 실제 하드웨어 주기처럼 각자 다르게
        self.dt = 1 / 60
        self.create_timer(self.dt, self._physics)
        self.create_timer(1 / 15, self._camera)
        self.create_timer(1 / 10, self._scan)
        self.create_timer(1 / 5, self._topdown)
        self.create_timer(1.0, self._status)
        self.get_logger().info("mini_sim 시작. /cmd_vel 을 기다리는 중 (obstacles_enabled=%s)"
                               % self.obstacles)

    # ----------------------------------------------------------------- 콜백
    def _on_params(self, params):
        # param set 이 "적용되기 직전"에 호출됨. successful=False 를 돌려주면 변경이 거부됨.
        # 다른 파라미터는 쓸 때마다 get_parameter 로 읽어서 따로 처리할 필요 없음.
        for p in params:
            if p.name == "obstacles_enabled":
                self.obstacles = bool(p.value)
                self.get_logger().info("장애물 %s" % ("ON" if self.obstacles else "OFF"))
        return SetParametersResult(successful=True)

    def _on_cmd(self, msg):
        # 저장만 함 (Lab 2와 같은 "콜백은 저장, 계산은 타이머" 패턴). 받은 시각도 같이 기록
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now() # 시간 저장이유: _physics에서 명령 끊겼는지 판단 

    def _on_reset(self, req, res):
        self._reset()
        self.get_logger().info("리셋")
        return res

    def _reset(self):
        self.car.reset(*self.track.start_pose())
        self.ref.reset()
        self.trail = []

    # 장애물 OFF 면 빈 리스트 → LiDAR·충돌판정·그림 모두 박스가 "없는 것"처럼 동작
    @property
    def boxes(self):
        return self.box_list if self.obstacles else []

    # ----------------------------------------------------------------- 루프
    def _physics(self):
        # 1. 마지막 명령이 몇 초 전에 왔는지 계산
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        v, w = self.cmd
        # 2. 명령이 0.5초 넘게 끊겼거나 충돌 상태면 강제로 0 명령
        if age > self.get_parameter("cmd_timeout").value or self.ref.collision:
            v, w = 0.0, 0.0   # 명령이 끊기면 정지 (실차에서도 꼭 넣어야 하는 안전장치)
        # 3. 차를 dt(1/60초)만큼 움직임. 반환값 = 실제 요레이트 (조향 한계·지연 반영 후)
        yaw_rate = self.car.step(v, w, self.dt)
        # 4. 채점: 이탈 횟수·바퀴 수·충돌 갱신
        self.ref.update(self.car, self.boxes, self.dt)
        # 5. 충돌 에러 로그는 충돌 순간 한 번만 (매 60 Hz마다 찍히지 않게 _warned 플래그)
        if self.ref.collision and not getattr(self, "_warned", False):
            self.get_logger().error("충돌! /sim/reset 서비스로 리셋하세요")
            self._warned = True
        if not self.ref.collision:
            self._warned = False
        # 6. 궤적: 2 cm 이상 움직였을 때만 점 추가, 최근 1500개만 유지 (메모리 제한)
        if len(self.trail) == 0 or math.hypot(self.car.x - self.trail[-1][0],
                                              self.car.y - self.trail[-1][1]) > 0.02:
            self.trail.append((self.car.x, self.car.y))
            self.trail = self.trail[-1500:]
        # 7. 새 위치를 /odom + TF 로 알림 (물리와 같은 60 Hz)
        self._publish_odom(yaw_rate)

    def _publish_odom(self, yaw_rate):
        # 같은 위치 정보를 두 형태로 발행: TF(좌표계 관계, rviz2용) + Odometry(위치+속도 메시지)
        now = self.get_clock().now().to_msg()
        # yaw(2D 회전각) → 쿼터니언. z축 회전만 있으면 (x=0, y=0, z=sin(θ/2), w=cos(θ/2))
        qz, qw = math.sin(self.car.yaw / 2), math.cos(self.car.yaw / 2)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.car.x
        t.transform.translation.y = self.car.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.car.x
        o.pose.pose.position.y = self.car.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = self.car.v
        o.twist.twist.angular.z = yaw_rate
        self.pub_odom.publish(o)

    def _camera(self):
        # 현재 차 위치·방향에서 앞쪽 0.15~0.65 m 바닥을 위에서 본 120×160 흑백 numpy 배열 생성
        img = self.cam.render(self.track, self.car.x, self.car.y, self.car.yaw, self.rng,
                              noise=self.get_parameter("camera_noise").value)
        # numpy → sensor_msgs/Image 포장. step = 한 행의 바이트 수 (mono8 은 1바이트/픽셀 → width)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = img.shape
        msg.encoding = "mono8"
        msg.step = img.shape[1]
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def _scan(self):
        # 360빔 거리 배열 (박스에 안 맞은 빔은 inf). 각도 0 = 차 정면, 반시계 +
        ranges = self.lidar.scan(self.car.x, self.car.y, self.car.yaw, self.boxes, self.rng)
        self._last_ranges = ranges   # _topdown 이 빨간 점을 그릴 때 재사용 (다시 계산 안 하려고)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = self.lidar.angle_min
        msg.angle_increment = self.lidar.angle_increment
        msg.angle_max = self.lidar.angle_min + self.lidar.angle_increment * (self.lidar.n - 1)
        msg.scan_time = 0.1
        msg.range_min = self.lidar.range_min
        msg.range_max = self.lidar.range_max
        msg.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(msg)

    def _topdown(self):
        if not self.get_parameter("publish_topdown").value:
            return
        # 1. 끄는 옵션이 켜져 있으면 아무것도 안 함 (그림 그리기가 가장 무거운 작업이라)
        # 2. 마지막 LiDAR 결과 중 실제로 뭔가에 맞은 빔(isfinite)만 골라
        # 3. 차 기준 (거리, 각도) → 지도 기준 (x, y) 로 변환 = 빨간 점 위치
        pts = []
        r = getattr(self, "_last_ranges", None)   # 첫 _scan 전이면 None
        if r is not None:
            ok = np.isfinite(r)
            a = self.car.yaw + self.lidar.angles[ok]   # 빔 각도(차 기준) + 차 방향 = 지도 기준 각도
            pts = list(zip(self.car.x + r[ok] * np.cos(a), self.car.y + r[ok] * np.sin(a)))
        # 4. 트랙 배경 + 박스 + 궤적 + LiDAR 점 + 차(초록 원, 노란 방향선)를 RGB 배열로 그림
        img = self.view.draw(self.car, self.boxes, self.trail, pts)
        # 5. rgb8 은 픽셀당 3바이트 → step = width * 3
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.pub_top.publish(msg)

    def _status(self):
        # 채점 결과 dict 를 JSON 문자열로 (커스텀 메시지 타입 없이 여러 값을 보내는 간단한 방법)
        self.pub_status.publish(String(data=json.dumps(self.ref.summary(), ensure_ascii=False)))


def main():
    rclpy.init()
    node = MiniSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
