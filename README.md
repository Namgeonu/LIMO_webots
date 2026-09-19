# LIMO_webots — Webots 에서 AgileX LIMO 한 대 운용

LIMO 단독 검증용 프로젝트다. 10×10 m 아레나에
`LimoFourDiff` 한 대를 놓고 `webots_ros2_driver + ros2_control + Nav2` 로 움직인다.
Ubuntu 22.04 / ROS 2 Humble / Webots R2025a.

## 실행

```bash
# 터미널 1
webots ~/LIMO_webots/worlds/limo_10x10.wbt

# 터미널 2
source ~/LIMO_webots/setup_env.sh
ros2 launch ~/LIMO_webots/limo.launch.py
```

Nav2 가 뜨는 데 launch 후 약 12 초 걸린다(`TimerAction`). 그 뒤 목표를 보낸다:

```bash
source ~/LIMO_webots/setup_env.sh
ros2 action send_goal /limo/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

확인용:
```bash
ros2 topic hz /limo/scan            # 라이다 (약 3 Hz)
ros2 topic echo /limo/odom --once   # 지상 진실 odom (gt_odom)
ros2 run tf2_ros tf2_echo limo/odom limo/laser_link   # z ≈ 0.111 이어야 한다
```

빌드는 필요 없다. 표준 apt 패키지만 쓴다 (`ros-humble-webots-ros2`, `ros-humble-navigation2`,
`ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-robot-state-publisher`).

## 구성

```
LIMO_webots/
├── limo.launch.py            Supervisor + 드라이버 + ros2_control 스포너 + static TF + Nav2 (한 대)
├── setup_env.sh              humble 소스 + PYTHONPATH(controllers)
├── worlds/limo_10x10.wbt     10x10 아레나, LimoFourDiff "limo", Ros2Supervisor
├── protos/LimoFourDiff.proto 로봇 모델 (+ meshes/ 차체·바퀴 dae)
├── controllers/limo.urdf     TF 골격 + 장치 선언 + ros2_control 하드웨어 (__NS__ 템플릿)
├── controllers/gt_odom.py    GPS+InertialUnit 기반 odom/TF 플러그인
└── config/                   limo_nav2_params.yaml, limo_ros2control.yaml
```

## 모델 메모

- 로봇 모델은 AgileX 공식 Gazebo `limo_four_diff` 를 기준으로 한다. 라이다는 지면 위 0.111 m
  (`base_link` 기준 z=−0.034), 최소 거리 0.3 m.
- 라이다 값은 proto(실제 광선 위치)와 URDF(스캔을 해석하는 TF) **두 곳이 반드시 같아야 한다.**
  한쪽만 바꾸면 costmap 에 장애물이 엉뚱한 높이로 찍히거나 전부 폐기된다.
- `gt_odom.py` 는 GPS + InertialUnit 으로 지상 진실 `odom` 과 `odom→base_link` TF 를 낸다
  (`base_link` 는 지면 위 0.145 m, `base_footprint` 는 그 아래 −0.145).

## 로봇을 더 놓으려면

월드에 `LimoFourDiff { name "limo_b" ... }` 를 추가하고 `ros2 launch ... robots:=limo,limo_b` 로
띄우면 된다. URDF/Nav2 params 는 로봇 이름별로 `/tmp/{name}.urdf`, `/tmp/{name}_nav2_params.yaml`
에 생성된다.
