#!/bin/bash
# LIMO_webots 공용 환경 설정
# 사용: source ~/LIMO_webots/setup_env.sh
#
# 커스텀 ROS 패키지 빌드가 필요 없다. LIMO 스택(webots_ros2_driver, ros2_control, Nav2,
# tf2_ros, robot_state_publisher)은 전부 apt 로 설치되는 표준 패키지다.
# PYTHONPATH 는 webots_ros2_driver 가 controllers/gt_odom.py 를 플러그인(gt_odom.GTOdom)으로
# 찾기 위해 필요하다 (limo.urdf 의 <plugin type="gt_odom.GTOdom"/>).
source /opt/ros/humble/setup.bash
export PYTHONPATH="$HOME/LIMO_webots/controllers:$PYTHONPATH"
