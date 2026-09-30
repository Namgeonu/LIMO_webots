#!/bin/bash
# LIMO_webots_omni 공용 환경 설정
# 사용: source ~/LIMO_webots_omni/setup_env.sh
#
# 커스텀 ROS 패키지 빌드가 필요 없다. LIMO 스택(webots_ros2_driver, Nav2 + nav2_mppi_controller,
# tf2_ros, robot_state_publisher)은 전부 apt 로 설치되는 표준 패키지다.
# PYTHONPATH 는 webots_ros2_driver 가 controllers/ 의 파이썬 플러그인을 찾기 위해 필요하다
# (limo.urdf 의 <plugin type="mecanum_driver.MecanumDriver"> 와 <plugin type="gt_odom.GTOdom"/>,
#  그리고 mecanum_driver 가 import 하는 mecanum_kinematics.py).
source /opt/ros/humble/setup.bash
# 이 스크립트가 있는 폴더 기준으로 잡는다 (폴더 이름·위치가 바뀌어도, 차동 버전 ~/LIMO_webots 와 섞이지 않게).
LIMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$LIMO_ROOT/controllers:$PYTHONPATH"
