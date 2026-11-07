"""Lab 2 ★ 내가 채우는 파일 ★  turtlesim 거북이를 목표 좌표까지 P 제어로 보내기

  ros2 run turtlesim turtlesim_node                     (터미널 1)
  ros2 run mini_av turtle_goal --ros-args -p goal_x:=2.0 -p goal_y:=8.0   (터미널 2)

구독  /turtle1/pose     turtlesim/msg/Pose   (x, y, theta)
발행  /turtle1/cmd_vel  geometry_msgs/Twist

ROS 2 노드의 뼈대 4가지가 다 들어있다: 파라미터, 구독, 발행, 타이머.

전체흐름 !!! 
rclpy.init → __init__ (파라미터·구독·발행·타이머 등록) → spin 시작
   ├─ pose 도착 (~62Hz)  → on_pose: self.pose 갱신
   └─ 0.05초마다 (20Hz) → control_loop: 계산 → publish(cmd)

"""
import math

import rclpy #ros2 파이썬 라이브러리 
from geometry_msgs.msg import Twist # 속도 명령 메시지 
from rclpy.node import Node # 모든 노드가 상속받는 기본 클래스 
from turtlesim.msg import Pose # turtlesim의 위치 메시지 


class TurtleGoal(Node):
    def __init__(self):
        super().__init__("turtle_goal") # 노드 이름을 tirtle_goal로 설정 ! 
        # 1) 파라미터 선언 (실행할 때 --ros-args -p goal_x:=2.0 으로 바꿀 수 있다)
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 8.0)
        # 아래 두개는 제어게인 - 얼마나 세게 반응할지를 정함 
        self.declare_parameter("k_lin", 1.0) 
        self.declare_parameter("k_ang", 4.0)

        self.pose = None
        self.arrived = False  # "도착!" 로그를 한 번만 찍기 위한 플래그
        # 2) 구독: 메시지가 올 때마다 on_pose 호출됨 + self.create_subscription(타입, 토픽이름, 콜백함수, 큐 크기)
        # 큐는 처리가 지연될 때 최대 10개까지 쌓아둔단뜻. 
        self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 10)

        # 3) 발행자 ( /turtle1/cmd_vel로 Twist 메시지를 보낼 수 있는 송신기를 만듦 ) 
        # 실제 전송은 self.pub.publish(cmd)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # 4) 타이머: 20 Hz 로 control_loop 실행
        # ( 0.05초 마다 = 1초에 20번 => 20hz 마다 control_loop 호출)
        self.create_timer(0.05, self.control_loop)

    def on_pose(self, msg):
        self.pose = msg # 받은 위치를 저장만함. -> 계산은 타이머에서 따로 함
    # 데이터가 들어오는 속도(센서)와 명령을 내리는 속도(제어)를 서로 다르게 !!! 
    # pose(=차량or거북이에서 라즈베리파이로 위치 정보를 보내는것)는 62hz(1초에 62번) / 제어는 20hz면 충분. 

    # 결과를 다시 보고 명령을 고치는 구조 = 피드백 제어 
    def control_loop(self):
        if self.pose is None:
            return
        # gx, gy 목표좌표 - 파라미터를 매번 새로 읽기에 실행 중에 param set으로 바꾸면 바로 반영됨
        gx = self.get_parameter("goal_x").value
        gy = self.get_parameter("goal_y").value
        cmd = Twist() # 모든값이 0인 속도 명령. 아무것도 안채우고 보내면 멈춰라는 뜻임. 

        # TODO 1) 목표까지 거리 dist = hypot(gx - x, gy - y)
        # hypot : 직각삼각형의 두 변의 길이를 이용해 빗변(Hypotenuse)의 길이를 구하거나, 원점에서 특정 좌표까지의 유클리드 거리(Euclidean distance)를 계산하는 프로그래밍 함수
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy) 

        # TODO 2) 목표 방향 target = atan2(gy - y, gx - x)
        target = math.atan2(dy, dx)

        # TODO 3) 방향 오차 = target - theta  →  -pi~pi 로 정규화 (atan2(sin, cos) 트릭)
        err = target - self.pose.theta
        err = math.atan2(math.sin(err) , math.cos(err))  # 정규화를 안하면 -170 -> 170을 340도 돌아서 해결해버림. 이건 그냥 20도만 우측으로 돌면 해결되는 문제 !!! 정규화가 중요하다 

        # TODO 4) cmd.angular.z = k_ang * 방향오차
        #         cmd.linear.x  = k_lin * dist   (방향이 많이 틀어졌으면 줄이거나 0)
        # k_lin (gain) : 하이퍼 파라미터 => 코드에 박아 두지 않고 self.get_parameter("k_ang") 같은 ROS 파라미터로 받음. 
        k_lin = self.get_parameter("k_lin").value
        k_ang = self.get_parameter("k_ang").value

        cmd.angular.z = k_ang * err
        # cos(err): 정면이면 1, 90도 이상 틀어지면 0 → 제자리에서 먼저 방향부터 돌림. 속도 상한 2.0
        cmd.linear.x = min(k_lin * dist, 2.0) * max(0.0, math.cos(err))

        # TODO 5) dist < 0.1 이면 멈추고 "도착!" 로그 (self.get_logger().info)
        if dist < 0.1:
            cmd = Twist()  # 전부 0 = 정지
            if not self.arrived:  # 20Hz로 로그가 도배되지 않게 처음 한 번만 출력
                self.get_logger().info(f"도착! ({self.pose.x:.2f}, {self.pose.y:.2f})")
                self.arrived = True
        else:
            self.arrived = False  # param set으로 목표가 바뀌면 다시 출발

        self.pub.publish(cmd)


def main():
    rclpy.init() # ros2 시작 
    node = TurtleGoal() # 노드 생성 -> __init__에서 구독/발행/타이머 등록 
    try:
        # while true로 무한루프를 짤 필요없음 !! spin이 해준다 
        rclpy.spin(node) # 무한 대기하면서 콜백과 타이머 호출 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
