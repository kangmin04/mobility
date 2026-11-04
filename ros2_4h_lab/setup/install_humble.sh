#!/usr/bin/env bash
# Ubuntu 22.04 (VMware) 에 ROS 2 Humble + 실습에 필요한 패키지 설치
# 사용법:  bash ~/ros2_4h_lab/setup/install_humble.sh
# 시간: 인터넷 속도에 따라 15~30분. ★ 4시간 실습 전에 미리 돌려둘 것 ★
set -euo pipefail

. /etc/os-release
if [ "${VERSION_ID}" != "22.04" ]; then
  echo "경고: Ubuntu ${VERSION_ID} 입니다. Humble 은 22.04 용입니다 (24.04 면 Jazzy 사용)." >&2
  exit 1
fi

sudo apt update
sudo apt install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-turtlesim \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rqt-image-view \
  ros-humble-rqt-graph \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-numpy python3-opencv \
  git

LINE='source /opt/ros/humble/setup.bash'
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc
WS='[ -f ~/ros2_4h_lab/ros2_ws/install/setup.bash ] && source ~/ros2_4h_lab/ros2_ws/install/setup.bash'
grep -qxF "$WS" ~/.bashrc || echo "$WS" >> ~/.bashrc
# 같은 네트워크의 다른 ROS 2 장치(팀원 PC, 실차)와 토픽이 섞이지 않게
grep -q ROS_DOMAIN_ID ~/.bashrc || echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc

echo
echo "설치 끝. 새 터미널을 열거나 'source ~/.bashrc' 후 확인:"
echo "  ros2 run demo_nodes_cpp talker"
