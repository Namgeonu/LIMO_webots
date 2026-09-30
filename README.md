# LIMO_webots — Omni_mode (메카넘 / 전방향)

## 1. 프로젝트 개요

Webots 시뮬레이터에서 AgileX LIMO 를 **메카넘(전방향) 모드**로 구동하는 프로젝트다.
10×10 m 아레나에 LIMO 한 대를 놓고, ROS 2 + Nav2 로 자율 주행한다.

- 목표 좌표를 주면 Nav2 가 경로를 계획하고, **헤딩을 유지한 채 옆·대각으로도 이동**한다.
- 위치 추정은 시뮬레이터 진실값(GPS + IMU)을 쓴다. 구동·경로 계획 검증이 목적이다.
- 환경: Ubuntu 22.04 / ROS 2 Humble / Webots R2025a

```
├── limo.launch.py                 Supervisor + 드라이버 + static TF(map->odom) + Nav2
├── setup_env.sh                   ROS 환경 + PYTHONPATH(controllers) 설정
├── worlds/limo_10x10.wbt          10x10 아레나, 메카넘 바퀴 마찰 설정
├── protos/LimoMecanum.proto       메카넘 LIMO 모델 (사용)
├── protos/LimoFourDiff.proto      차동 LIMO 원본 모델 (비교용 보존)
├── controllers/limo.urdf          TF 골격 + 드라이버 플러그인 선언
├── controllers/mecanum_driver.py  cmd_vel(vx, vy, wz) -> 역기구학 -> 바퀴 모터 4개
├── controllers/mecanum_kinematics.py  메카넘 IK/FK 순수 함수
├── controllers/gt_odom.py         GPS + IMU 기반 odom / TF
└── config/limo_nav2_params.yaml   Nav2 설정 (MPPI, Omni 모델)
```

## 2. 차륜 구동(main) 대비 변경점

| 층 | main (차동) | Omni_mode (메카넘) |
|---|---|---|
| 물리 | 일반 바퀴 마찰 | 바퀴 재질을 대각으로 나누고 ±45° 비대칭 마찰 적용 (Webots 공식 KUKA youBot 방식) |
| 구동 | `ros2_control` + `diff_drive_controller` | 파이썬 플러그인 `mecanum_driver.py` 하나 (IK, 속도 제한, 명령 타임아웃, joint_states) |
| 기구학 | 차동 (vx, wz) | 메카넘 (vx, **vy**, wz). ros2_controllers `mecanum_drive_controller` 와 같은 식 |
| odom | 전진 속도만 | 측면 속도(`twist.linear.y`)도 발행 |
| Nav2 | DWB, 목표 헤딩 무시 | **MPPI (`motion_model: Omni`)**, 이동 중 헤딩 유지, 목표 헤딩 맞춤 |

- `config/limo_ros2control.yaml` 과 URDF 의 `<ros2_control>` 블록, launch 의 컨트롤러 스포너는 삭제했다.
- 차체·센서·치수·질량은 원본 LIMO 모델과 같다.

## 3. 설치

빌드는 필요 없다. apt 패키지만 설치하면 된다.

```bash
sudo apt install ros-humble-webots-ros2 ros-humble-navigation2 \
                 ros-humble-nav2-mppi-controller ros-humble-robot-state-publisher
```

저장소를 `Omni_mode` 브랜치로 받는다. 차동 버전(main)과 섞이지 않게 폴더를 따로 두는 것을 권장한다.

```bash
git clone -b Omni_mode https://github.com/Namgeonu/LIMO_webots.git ~/LIMO_webots_omni
```

## 4. 실행

**시뮬레이션 + Nav2 실행**

```bash
# 터미널 1: Webots 월드
webots ~/LIMO_webots_omni/worlds/limo_10x10.wbt

# 터미널 2: 드라이버 + Nav2
source ~/LIMO_webots_omni/setup_env.sh
ros2 launch ~/LIMO_webots_omni/limo.launch.py
```

Nav2 는 launch 후 약 12 초 뒤에 뜬다. 로봇은 (-3, 0) 에서 +x 를 보고 시작한다.

**직접 속도 명령 (동작 확인)** — 새 터미널에서

```bash
source ~/LIMO_webots_omni/setup_env.sh
ros2 topic pub -r 20 /limo/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"          # 좌측 게걸음
ros2 topic pub -r 20 /limo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1}}"  # 좌전 대각
```

`Ctrl+C` 로 멈추면 0.5 초 뒤 로봇이 정지한다.

**Nav2 목표 보내기**

```bash
source ~/LIMO_webots_omni/setup_env.sh
# 헤딩 유지한 채 왼쪽 2 m 이동
ros2 action send_goal /limo/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

목표 `orientation` 을 시작 헤딩과 다르게 주면, 이동 중에는 헤딩을 유지하다가 목표 0.5 m 안에서 목표 헤딩으로 돌린다.

> `setup_env.sh` 는 자기 폴더의 `controllers/` 를 PYTHONPATH 에 넣는다.
> 차동 버전 폴더의 `setup_env.sh` 를 source 하면 `mecanum_driver` 를 찾지 못해 로봇이 움직이지 않는다.
