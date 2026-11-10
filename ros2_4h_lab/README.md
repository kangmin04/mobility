# ROS 2 자율주행 4시간 실습 (Jazzy · VMware Ubuntu 24.04 · SSH)

> 2026-11-10: 팀 배포판을 Humble에서 **Jazzy**로 바꿨다. VM은 Ubuntu 24.04, Docker는 `setup/docker-compose.yml`(`tiryoh/ros2-desktop-vnc:jazzy`)을 쓴다. 예전 Humble 컨테이너에서 빌드한 `ros2_ws/build install log`는 지우고 다시 빌드한다.

> 목표: 4시간 뒤에 **"카메라 → 차선 인식 → 판단(FSM) → PID 조향 → /cmd_vel"** 파이프라인과 **LiDAR 정지**를 직접 짜서 시뮬레이터에서 돌리고, rqt_graph·rosbag으로 증거를 남길 수 있다.
> 크누모 대회 구조(PiCar-X, 차선 추종 + LiDAR 장애물)에 맞춘 축소판이다. 노드 이름과 토픽 구조를 대회 코드로 그대로 옮겨 쓸 수 있다.

---

## 0. 4시간 시간표

| 시간 | 실습 | 하는 일 | 끝났는지 확인하는 기준 |
|---|---|---|---|
| (사전) | 설치 | `setup/install_jazzy.sh` 실행 (15~30분, **4시간에 포함 안 함**) | `ros2 run demo_nodes_cpp talker` 가 실행됨 |
| 0:00–0:15 | **Lab 0** 환경 | 파일 옮기기, 빌드, GUI 띄우는 방법 정하기 | `ros2 pkg list \| grep mini_av` 결과가 나옴 |
| 0:15–0:55 | **Lab 1** ROS 2 개념 + CLI | turtlesim으로 node / topic / message / service / parameter / rqt_graph | 아래 Lab 1 질문 5개에 답할 수 있음 |
| 0:55–1:40 | **Lab 2** 첫 노드 | `turtle_goal.py`: 파라미터, 구독, 발행, 타이머, P 제어 | 거북이가 `goal_x, goal_y`에 가서 멈춤 |
| 1:40–1:50 | 휴식 | | |
| 1:50–2:10 | **Lab 3-0** 시뮬레이터 | mini_sim을 teleop으로 직접 몰아보고, 정답 파이프라인 구경 | `/camera/image_raw`, `/scan` 영상을 봄 |
| 2:10–3:05 | **Lab 3** 차선 추종 | `find_lane_center` + `steering_control` 직접 구현, 게인 튜닝 | `offline_check.py` PASS, ROS에서 2바퀴 이탈 0 |
| 3:05–3:35 | **Lab 4** LiDAR + FSM | `front_distance` + `decide` 구현, 실행 중 장애물 켜고 끄기 | 박스 앞 정지 → 치우면 다시 출발 |
| 3:35–4:00 | **Lab 5** 실무 도구 | launch, params.yaml, rosbag record/play, rqt_graph 캡처 | `run01` bag 파일과 그래프 캡처 1장 |

남는 시간이 있으면 맨 아래 **확장 과제**로 넘어간다.

### 폴더 구조

```
ros2_4h_lab/
├── README.md                  ← 이 문서
├── setup/install_jazzy.sh     ← ROS 2 Jazzy 설치 스크립트
├── tools/offline_check.py     ← ROS 없이 알고리즘만 채점 (numpy만 필요)
└── ros2_ws/src/mini_av/       ← ROS 2 패키지 (ament_python)
    ├── mini_av/
    │   ├── my_algorithms.py   ★ Lab 3·4 에서 내가 채우는 파일
    │   ├── turtle_goal.py     ★ Lab 2 에서 내가 채우는 파일
    │   ├── solutions.py       정답 예시 (막히면 보기)
    │   ├── turtle_goal_solution.py
    │   ├── sim_node.py        2D 시뮬레이터 (카메라·LiDAR·차량 모델)
    │   ├── lane_detect_node.py / obstacle_node.py / controller_node.py
    │   └── core.py            시뮬레이터 내부 계산 (ROS 의존성 없음)
    ├── launch/lane_follow.launch.py, sim.launch.py
    └── config/params.yaml
```

### 최종적으로 만들 ROS 2 그래프

```
                 /camera/image_raw            /lane_error
 [mini_sim] ───────────────────▶ [lane_detect] ──────────▶ ┌────────────┐   /cmd_vel
     ▲  │              /scan                /obstacle_dist  │ controller │ ──────────┐
     │  └──────────────────────▶ [obstacle] ─────────────▶ │ (FSM + PD) │           │
     │                                                      └────────────┘           │
     └──────────────────────────────────────────────────────────────────────────────┘
```
대회 로드맵의 노드 구성 초안(`camera_node → lane_detect_node → decision_node → control_node`)과 같은 모양이다. 실차에서는 `mini_sim` 자리에 카메라·LiDAR 드라이버와 모터 노드가 들어간다.

---

## Lab 0. 환경 준비 (15분)

### 0-1. 파일을 VM으로 옮기기 (Mac 터미널에서)

```bash
# VM IP 확인: VM 안에서  ip -4 addr | grep inet
scp -r ~/Desktop/personal-study/mobility/ros2_4h_lab <user>@<VM_IP>:~/
ssh <user>@<VM_IP>
```

> 코드 편집은 **VS Code + Remote-SSH 확장**이 가장 편하다. `~/ros2_4h_lab` 폴더를 열면 Mac에서 VM 파일을 바로 고칠 수 있다.

### 0-2. 설치 (아직 안 했다면, VM에서)

```bash
bash ~/ros2_4h_lab/setup/install_jazzy.sh
source ~/.bashrc
```

### 0-3. 빌드

```bash
cd ~/ros2_4h_lab/ros2_ws
colcon build --symlink-install     # --symlink-install: .py 고치고 다시 빌드 안 해도 반영됨
source install/setup.bash
ros2 pkg executables mini_av       # sim, lane_detect, obstacle, controller, turtle_goal ...
```

> **source 규칙**: 새 터미널마다 `/opt/ros/jazzy/setup.bash`(ROS 본체)와 `install/setup.bash`(내 패키지)를 읽어야 한다. 설치 스크립트가 `~/.bashrc`에 둘 다 넣어 두었다. 새 노드를 추가하거나 `setup.py`를 고쳤을 때만 `colcon build`를 다시 한다.

### 0-4. GUI 창(turtlesim, rqt, rviz2) 띄우는 방법 — SSH라서 필요

SSH 터미널에는 화면이 없다. 셋 중 하나를 고른다.

| 방법 | 방법 | 장단점 |
|---|---|---|
| **A. VM 화면에 띄우기 (추천)** | VM 창에서 로그인해 둔 뒤, SSH 터미널에서 `export DISPLAY=:0` 하고 실행 | 빠르다. 로그인 화면 톱니바퀴에서 **"Ubuntu on Xorg"** 를 골라야 SSH에서 띄울 수 있다 (Wayland면 권한 오류) |
| B. VM 창 안의 터미널 | GUI 명령만 VM 창의 터미널에서 실행 | 설정이 필요 없다 |
| C. Mac으로 X11 포워딩 | Mac에 XQuartz 설치 → `ssh -Y <user>@<VM_IP>` | rqt는 괜찮지만 rviz2는 느리거나 깨질 수 있다 |

**같은 `ROS_DOMAIN_ID`면 SSH 터미널과 VM 창 터미널의 노드끼리 바로 통신된다.** 그래서 방법 B도 문제없다.

---

## Lab 1. ROS 2 핵심 개념 + CLI (40분)

### 개념 5개만 먼저

| 개념 | 한 줄 | 자율주행에서의 예 |
|---|---|---|
| **Node** | 한 가지 일을 하는 프로세스 | 차선 인식 노드, 제어 노드 |
| **Topic** | 노드끼리 계속 흘려보내는 데이터 통로 (1:N, 비동기) | `/camera/image_raw`, `/cmd_vel` |
| **Message** | 토픽에 실리는 데이터 형식 | `sensor_msgs/Image`, `geometry_msgs/Twist` |
| **Service** | 요청하면 한 번 응답 (동기) | 시뮬레이터 리셋, 모드 변경 |
| **Parameter** | 노드의 설정값. 실행 중에도 바꿀 수 있음 | PID 게인, 속도, threshold |

### 실습 (터미널 3개)

```bash
# T1 (GUI)
ros2 run turtlesim turtlesim_node
# T2 (GUI 아님. 키보드로 조종. 이 터미널을 클릭해 둔 상태로 방향키)
ros2 run turtlesim turtle_teleop_key
```

```bash
# T3: 관찰
ros2 node list                         # 어떤 노드가 떠 있나
ros2 node info /turtlesim              # 이 노드의 구독/발행/서비스/파라미터
ros2 topic list -t                     # 토픽과 메시지 타입
ros2 topic echo /turtle1/pose          # 거북이 위치가 계속 흘러나옴 (Ctrl+C)
ros2 topic hz /turtle1/pose            # 몇 Hz로 오나  ← 대회에서 카메라 FPS 확인할 때 쓰는 명령
ros2 interface show geometry_msgs/msg/Twist   # Twist 안에 뭐가 들었나

# 키보드 대신 명령으로 직접 발행
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.5}, angular: {z: 1.0}}"     # 원을 그림. Ctrl+C로 멈춤

# 서비스
ros2 service list
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
ros2 service call /clear std_srvs/srv/Empty

# 파라미터
ros2 param list /turtlesim
ros2 param set /turtlesim background_r 200
ros2 service call /clear std_srvs/srv/Empty  # 배경색 변경 반영

# 그래프 (GUI)
rqt_graph
```

### 확인 질문 (답 못 하면 다시 보기)

1. `turtle_teleop_key`와 `turtlesim_node`는 어떤 토픽으로 연결되어 있나? 메시지 타입은?
2. `/turtle1/pose`는 몇 Hz로 오나?
3. `Twist`의 `linear.x`와 `angular.z`는 각각 무슨 의미이고 단위는?
4. 거북이를 하나 더 만드는 건 왜 토픽이 아니라 서비스일까?
5. 노드를 다시 빌드하지 않고 동작을 바꾸려면 무엇을 쓰면 되나?

<details><summary>답</summary>

1. `/turtle1/cmd_vel`, `geometry_msgs/msg/Twist`
2. 약 62 Hz
3. 전진 속도 [m/s], 회전 속도(요레이트) [rad/s]. 반시계 방향이 +
4. "한 번 요청하고 성공/실패 응답을 받는" 일이라서. 토픽은 계속 흐르는 데이터용이다
5. 파라미터 (`ros2 param set`, launch 파일, yaml)
</details>

---

## Lab 2. 첫 노드: 거북이를 목표점으로 (45분)

ROS 2 Python 노드는 거의 항상 **파라미터 선언 → 구독 → 발행 → 타이머 루프** 4가지로 이루어진다. `turtle_goal.py`에 뼈대가 다 있고, 제어 계산(`control_loop`)만 비어 있다.

```bash
# 파일 열기 (VS Code Remote-SSH 추천)
nano ~/ros2_4h_lab/ros2_ws/src/mini_av/mini_av/turtle_goal.py
```

**할 일**: TODO 1~5 채우기. 핵심 아이디어:
- 목표 방향 = `atan2(gy - y, gx - x)`
- 방향 오차 = 목표 방향 − 현재 방향 → `atan2(sin(e), cos(e))`로 −π~π 정규화 (안 하면 359° 돌아가는 버그)
- 회전 속도 = `k_ang × 방향 오차`, 전진 속도 = `k_lin × 거리` ← 이게 **P 제어**. Lab 3 조향 제어와 같은 원리다

```bash
# T1
ros2 run turtlesim turtlesim_node
# T2  (--symlink-install 로 빌드했으니 저장만 하면 됨)
ros2 run mini_av turtle_goal --ros-args -p goal_x:=2.0 -p goal_y:=8.0
# T3  실행 중에 목표 바꾸기
ros2 param set /turtle_goal goal_x 9.0
ros2 param set /turtle_goal k_ang 0.5     # 게인을 낮추면? 높이면? (10.0 도 해보기)
```

막히면 `ros2 run mini_av turtle_goal_solution`으로 정답 동작을 보고 `turtle_goal_solution.py`와 비교한다.

**체크포인트**: 거북이가 목표점에 가서 멈추고 "도착!" 로그가 뜬다. `k_ang`를 바꾸면 회전이 어떻게 달라지는지 말로 설명할 수 있다.

---

## Lab 3-0. 시뮬레이터 둘러보기 (20분)

`mini_sim`은 이 실습용으로 만든 가벼운 2D 시뮬레이터다 (Gazebo 불필요, VM에서도 가볍게 돈다).

- 트랙: 흰 바닥 + **검은 테이프 2줄**로 된 타원형 코스 (직선 2 m, 곡선 반지름 0.8 m, 차선 폭 0.35 m)
- 차량: PiCar-X 크기의 앞바퀴 조향 모델. 최대 조향각 30°, 모터·서보 반응 지연 있음
- 카메라: **이미 BEV 변환된** 160×120 흑백 영상 (위쪽 = 0.65 m 앞, 아래쪽 = 0.15 m 앞, 노이즈와 조명 불균일 있음)
- LiDAR: 360빔, 위쪽 직선 가운데에 0.2 m 박스 (`obstacles_enabled`로 켜고 끔)
- 평가: `/sim/status`에 바퀴 수, 랩타임, **이탈 횟수**, 보정 기록(랩타임 + 이탈×5초), 충돌 여부

```bash
# T1  시뮬레이터만
ros2 launch mini_av sim.launch.py
# T2  키보드로 몰아보기 (i 전진, j/l 회전, k 정지, q/z 속도 조절)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# T3  (GUI) 영상 보기. 왼쪽 위 드롭다운에서 /sim/topdown, /camera/image_raw 선택
ros2 run rqt_image_view rqt_image_view
# T4
ros2 topic echo /sim/status
ros2 topic hz /camera/image_raw            # 15 Hz
ros2 topic echo /scan --once | head -20    # LaserScan 구조: angle_min, angle_increment, ranges
ros2 param set /mini_sim obstacles_enabled true   # 박스 나타남 (topdown에서 빨간 점 = LiDAR 점)
ros2 service call /sim/reset std_srvs/srv/Empty
```

> teleop은 0.5초 넘게 명령이 없으면 차가 선다 (`cmd_timeout`). 실차에도 꼭 넣어야 하는 안전장치다.

다음으로 **정답 파이프라인**이 어떻게 도는지 먼저 본다 (T1, T2 Ctrl+C 후).

```bash
ros2 launch mini_av lane_follow.launch.py use_solution:=true
# 다른 터미널
ros2 run rqt_image_view rqt_image_view   # /lane/debug : 흰 세로줄 = 찾은 차선 중심, 회색 = 차량 중심
rqt_graph        # 위의 그래프 그림과 비교
ros2 topic echo /lane_error
```

---

## Lab 3. 차선 추종 직접 구현 (55분)

`my_algorithms.py`의 `find_lane_center`와 `steering_control`을 채운다.

### 3-1. 차선 인식 `find_lane_center` (세미나 Ⅱ의 4단계 그대로)

| 단계 | 할 일 | 생각할 점 |
|---|---|---|
| ① ROI | `img[int(H*0.4):int(H*0.8), :]` | 너무 멀리 보면 곡선에서 선이 화면 밖으로 나가고, 너무 가까우면 반응이 늦다 |
| ② BEV | (시뮬레이터가 이미 해줌) | 실차에서는 `cv2.getPerspectiveTransform` + `warpPerspective` |
| ③ 이진화 | `mask = roi < 임계값` | 영상 왼쪽이 일부러 어둡다. 고정 임계값 100이면 되나? |
| ④ 중심 계산 | 열별 히스토그램 → 왼쪽/오른쪽 선 위치 → 가운데 | 한쪽 선만 보이면? 둘 다 안 보이면? (= 중간평가 "차선 놓침 대응") |

반환값은 **미터 단위, 왼쪽이 +**. 열 번호를 미터로 바꿀 때 `cam.col_to_y(col)`을 쓴다.

### 3-2. 조향 제어 `steering_control` (PD)

```
angular.z = kp * error + kd * (error - prev_error) / dt
```
- **부호 먼저.** 차선 중심이 왼쪽(+)에 있으면 왼쪽(+)으로 돌아야 한다. 부호가 반대면 차가 바로 튕겨 나간다 (세미나에서 강조한 "오차 부호와 서보 영점 확인")
- 튜닝 순서: `kd=0`으로 두고 `kp`만 올려서 흔들리기 직전까지 → `kd`를 더해 흔들림 줄이기

### 3-3. 검사

```bash
# ① 빠른 채점 (ROS 없이 몇 초 만에 60초 주행 결과가 나옴)
cd ~/ros2_4h_lab
python3 tools/offline_check.py                 # PASS = 2바퀴 이상, 이탈 0, 충돌 없음
python3 tools/offline_check.py --kp 1.0        # 게인이 낮으면 어떻게 되나
python3 tools/offline_check.py --speed 0.6     # 빠르게 하면? 게인을 어떻게 바꿔야 하나

# ② ROS로 실제 주행
ros2 launch mini_av lane_follow.launch.py
ros2 run rqt_image_view rqt_image_view   # /sim/topdown, /lane/debug
ros2 topic echo /sim/status
ros2 param set /controller kp 3.0               # 달리는 중에 게인 바꾸기
ros2 param set /controller speed 0.5
```

**체크포인트**: `offline_check.py` PASS, ROS에서 2바퀴 이상 이탈 0. 튜닝한 (speed, kp, kd)와 결과(랩타임, 이탈 수)를 표로 적어 둔다 → 대회 "시행착오·개선" 항목의 증거가 된다.

| 시도 | speed | kp | kd | 랩타임 | 이탈 | 메모 |
|---|---|---|---|---|---|---|
| 1 | 0.35 | 5.0 | 0.3 | | | |
| 2 | | | | | | |

---

## Lab 4. LiDAR 정지 + FSM (30분)

### 4-1. `front_distance`
`ranges[i]`의 각도는 `angle_min + i * angle_increment` (0 = 정면). 정면 ±20° 안에서 `inf`가 아닌 값 중 최솟값을 돌려준다.

### 4-2. `decide` (유한 상태 기계)

```
            obstacle < 1.0            obstacle < 0.45
  FOLLOW ─────────────────▶ SLOW ─────────────────▶ STOP
    ▲  ▲                                              │
    │  └──────────── obstacle > 0.6 (히스테리시스) ────┘
    │ 차선 다시 찾음
  LOST  ◀── lane_error 가 NaN (1.5초 넘으면 정지)
```

- 정지 기준(0.45)과 출발 기준(0.6)을 다르게 두는 이유: 경계값 근처에서 노이즈 때문에 정지·출발을 반복하지 않게 하려고 (히스테리시스)
- 이 FSM 구조가 대회 8자 미션의 `FOLLOW → AT_CENTER → CHOOSE_BRANCH → … → STOP`으로 그대로 확장된다

```bash
python3 tools/offline_check.py --obstacle      # 박스 앞 STOP → 5초 뒤 박스 치움 → 다시 주행

ros2 launch mini_av lane_follow.launch.py obstacles:=true
ros2 topic echo /drive_state
ros2 topic echo /obstacle_distance
# 차가 박스 앞에 서면:
ros2 param set /mini_sim obstacles_enabled false    # 박스 치우기 → 다시 출발해야 함
```

**체크포인트**: 박스에 부딪히지 않고 서고, 치우면 다시 출발한다. `/drive_state`가 FOLLOW → SLOW → STOP → FOLLOW 순으로 바뀐다.

---

## Lab 5. 실무 도구: launch · 파라미터 · rosbag · 그래프 (25분)

### 5-1. launch와 params.yaml 읽어 보기
- `launch/lane_follow.launch.py`: 노드 4개를 한 번에 띄우고 `use_solution`, `obstacles` 인자를 받는다
- `config/params.yaml`: 튜닝한 값을 여기에 적는다. **Lab 3에서 찾은 게인으로 바꾸고** 다시 launch 해 보기 (`--symlink-install`이어도 yaml은 `colcon build`를 다시 해야 할 수 있다)

```bash
ros2 param dump /controller          # 지금 값을 yaml로 출력 → params.yaml 에 붙여넣기
```

### 5-2. rosbag으로 주행 기록 남기고 다시 보기

```bash
# 주행 중에
cd ~/ros2_4h_lab
ros2 bag record -o run01 /camera/image_raw /lane_error /cmd_vel /drive_state /sim/status
# 30초쯤 뒤 Ctrl+C
ros2 bag info run01

# 시뮬레이터, 제어 노드를 끄고 재생 → 인식 노드만 다시 돌려볼 수 있다
ros2 bag play run01
ros2 run mini_av lane_detect --ros-args -p use_solution:=true
```
> 이 방법이 핵심이다: **실차에서 한 번 녹화하면 집에서 인식 알고리즘을 계속 고쳐볼 수 있다.** (재생할 때는 bag에 담긴 `/lane_error`도 같이 나오니, 비교하려면 `ros2 bag play run01 --topics /camera/image_raw`)

### 5-3. 그래프 캡처 → 중간 발표 자료
`rqt_graph`에서 Nodes/Topics(all)로 바꾸고 캡처한다. 중간 세미나 평가표의 **"ROS2 통신 그래프"(25점)**에 그대로 쓸 수 있는 형태다.

---

## 확장 과제 (시간이 남거나 다음 날)

1. **차선 놓침 대응 강화**: `camera_noise`를 30으로 올리고 (`ros2 param set /mini_sim camera_noise 30.0`) 완주되는지. 안 되면 이진화를 바꾸거나 직전 값에 저역통과 필터를 건다
2. **곡선에서 감속**: `steering_control`의 출력이 크면 `speed`를 줄이게 `decide`를 바꾸기 → 보정 기록 단축
3. **OpenCV로 바꾸기**: `cv2.threshold(..., cv2.THRESH_OTSU)`, `cv2.moments`로 다시 구현하고 처리 시간 비교 (`lane_detect` 로그에 ms가 찍힌다)
4. **장애물 회피**: 정지 대신 차선 안에서 옆으로 비켜 지나가기 (목표 오프셋을 잠깐 바꾸기)
5. **RViz2**: `rviz2` → Fixed Frame `odom` → Add → LaserScan(`/scan`), Odometry(`/odom`), TF
6. **진짜 3D 시뮬레이터**: TurtleBot3 + Gazebo + Nav2 (`ros-jazzy-turtlebot3*`, `ros-jazzy-navigation2`). 대회 시뮬레이터가 Gazebo 계열이면 이쪽 경험이 도움이 된다

## 막힐 때

| 증상 | 원인 / 해결 |
|---|---|
| `Package 'mini_av' not found` | `source ~/ros2_4h_lab/ros2_ws/install/setup.bash` 안 함, 또는 빌드 안 함 |
| GUI가 `could not connect to display` | Lab 0-4 참고. `export DISPLAY=:0` + Xorg 세션, 또는 VM 창 터미널에서 실행 |
| 노드끼리 토픽이 안 보임 | 두 터미널의 `ROS_DOMAIN_ID`가 다름 (`echo $ROS_DOMAIN_ID`) |
| 차가 바로 트랙 밖으로 튕김 | 조향 부호가 반대. `error`가 +일 때 `angular.z`도 +인지 확인 |
| 차가 지그재그로 흔들림 | `kp`가 너무 큼 → 낮추거나 `kd`를 올림 |
| 곡선에서 바깥으로 밀림 | `kp`가 너무 작거나 속도가 너무 빠름 |
| 충돌 후 차가 안 움직임 | `ros2 service call /sim/reset std_srvs/srv/Empty` |
| 새 노드를 추가했는데 `ros2 run`이 못 찾음 | `setup.py`의 `entry_points`에 넣고 `colcon build` 다시 |

## 참고
- 공식 튜토리얼: https://docs.ros.org/en/jazzy/Tutorials.html (Beginner: CLI tools, Client libraries 부분이 Lab 1·2와 같은 범위)
- 대회 공부 로드맵: `../자율주행_공부_로드맵.md` (Ch 2·3·4·5가 이 실습과 연결됨)
