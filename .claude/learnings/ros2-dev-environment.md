# ROS 2 개발 환경: Docker vs VMware, GUI 창은 어디에 뜨나

## 2026-11-05 — ros2_4h_lab Lab 1을 Docker(`tiryoh/ros2-desktop-vnc:humble`)로 시작하면서

### turtlesim 창은 터미널이 아니라 **별도 GUI 창**에 뜬다
`ros2 run turtlesim turtlesim_node`는 **X11 디스플레이**에 창을 그리는 GUI 프로그램이다. 터미널에는 로그만 나오고, 창은 그 프로세스의 `DISPLAY` 환경변수가 가리키는 화면에 열린다. 그래서 "실행했는데 거북이가 안 보인다"는 대부분 **창이 다른 화면에 떠 있거나, 띄울 화면이 없는** 경우다.

| 실행 위치 | `DISPLAY` | 창이 뜨는 곳 |
|---|---|---|
| Docker VNC 컨테이너 (`ubuntu@<컨테이너ID>` 프롬프트) | `:1` (컨테이너 안 가상 X 서버 `Xtigervnc`) | **브라우저 http://localhost:6080** (noVNC, 비밀번호 `ubuntu`) |
| VMware VM 창 안의 터미널 | `:0` | VM 창 안 Ubuntu 바탕화면 |
| Mac → SSH → VM | 없음 → `could not connect to display` | `export DISPLAY=:0` 하면 VM 창에 뜸 |
| Mac → `ssh -Y` (XQuartz) | 포워딩된 디스플레이 | Mac 화면 (rviz2는 느림) |

```mermaid
flowchart LR
  T[turtlesim_node] -->|DISPLAY=:1| X[Xtigervnc :1<br/>컨테이너 안 가상 화면]
  X -->|rfbport 5901| W[websockify / noVNC :80]
  W -->|127.0.0.1:6080| B[Mac 브라우저]
```

**흔한 오해**: 로그가 `Spawning turtle [turtle1] at x=[5.54]...`에서 멈춰 있으면 실패한 게 아니다. 노드가 정상 실행 중이며, 창은 VNC 화면에 이미 떠 있다. `XDG_RUNTIME_DIR not set` 경고는 무시해도 된다.

```yaml
# ros2_4h_lab/setup/docker-compose.yml:5-13
services:
  ros:
    image: tiryoh/ros2-desktop-vnc:humble
    container_name: ros2-humble
    ports:
      - "127.0.0.1:6080:80"
    shm_size: "512m"
    volumes:
      - ../ros2_ws:/home/ubuntu/ros2_ws
```
`volumes` 때문에 Mac의 `ros2_ws`와 컨테이너의 `~/ros2_ws`가 같은 폴더다. `colcon build --symlink-install`로 빌드해 두면 Mac에서 `.py`만 고쳐 저장해도 바로 반영된다. 새 노드나 `setup.py`를 바꿀 때만 다시 빌드하면 된다.

### SSH에서 GUI가 안 뜰 때 (VMware)
- `could not connect to display` / `Could not load the Qt platform plugin "xcb"`: SSH 세션에 화면이 없다는 뜻이다. VM 창에서 로그인해 둔 뒤 `export DISPLAY=:0`을 실행한다.
- `Authorization required` / `cannot open display`: Wayland 세션이라서 그렇다. 로그인 화면의 톱니바퀴에서 **"Ubuntu on Xorg"**를 선택한다. `echo $XDG_SESSION_TYPE`이 `x11`이어야 한다.
- 가장 쉬운 우회 방법은 GUI 노드만 VM 창 안의 터미널에서 실행하는 것이다. **같은 `ROS_DOMAIN_ID`면** SSH 쪽 노드와 자동으로 통신한다.

### Docker를 계속 써도 되나? → 시뮬레이션까지는 OK, 실차 연결부터는 VMware(브리지)
| | Docker (Mac) | VMware |
|---|---|---|
| 준비 | `docker compose up -d` 한 줄 | Ubuntu + ROS 설치 |
| 망가졌을 때 | 컨테이너 재생성 | 직접 복구 |
| GUI 성능 | VNC라 GPU 가속 없음 → rviz2/Gazebo 느림 | 상대적으로 나음 |
| **실차와 DDS 통신** | 문제 있음 | 브리지 모드면 OK |

**왜 실차에서 Docker가 문제인가**: ROS 2(DDS)는 같은 네트워크에서 **멀티캐스트**로 다른 노드를 자동으로 찾는다. Mac의 Docker Desktop은 컨테이너를 별도 가상 네트워크(VM 안의 NAT) 뒤에 두기 때문에, 같은 와이파이의 PiCar-X(라즈베리파이)와 멀티캐스트 탐색이 잘 안 된다. VMware를 **브리지 모드**로 두면 VM이 공유기에서 IP를 직접 받아 로봇과 같은 LAN에 있는 PC처럼 동작한다.

**역할 정리**: 대회 때 노드가 도는 곳은 로봇(라즈베리파이)이다. Mac 쪽 환경은 코드 작성, 시뮬레이션, 모니터링(rqt/rviz2)을 하는 개발 PC다. 둘 다 Humble이면 `ros2_ws/src/mini_av`를 그대로 옮기면 된다.
