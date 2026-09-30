"""LIMO 메카넘(전방향) 기구학 — Webots/ROS 의존성이 없는 순수 함수 모듈.

mecanum_driver.py(시뮬 구동)가 import 해서 쓴다. 나중에 바퀴 odom 을 만들 때도
같은 식을 쓰도록 여기 한 곳에만 둔다.

[근거] 아래 역기구학(IK)은 다음 두 공식 구현과 식이 완전히 같다.
  1) ros-controls/ros2_controllers
     mecanum_drive_controller/src/mecanum_drive_controller.cpp (update_and_write_commands)
       FL = 1/R * (vx - vy - S*wz)      FR = 1/R * (vx + vy + S*wz)
       RR = 1/R * (vx - vy + S*wz)      RL = 1/R * (vx + vy - S*wz)
     S = sum_of_robot_center_projection_on_X_Y_axis = lx + ly
  2) cyberbotics/webots projects/robots/kuka/youbot/libraries/youbot_control/src/base.c
     base_move(): wheel1(FR), wheel2(FL), wheel3(RR), wheel4(RL) 에 같은 식.
     youBot 은 이 식 + 비대칭 마찰(FR·RL=InteriorWheelMat, FL·RR=ExteriorWheelMat)로
     Webots 안에서 전방향 이동한다. LimoMecanum.proto 는 그 재질 배치를 그대로 따른다.

좌표 규약: base_link 기준 x=정면, y=왼쪽, z=위 (REP-103).
바퀴 속도 부호: 양수 = 로봇을 앞으로 미는 방향 (LimoMecanum.proto 의 HingeJoint 축 +Y 기준).

LIMO 치수 (AgileX 기준 xacro wheelbase=0.2, track=0.13 / proto 바퀴 anchor ±0.1, ±0.065):
  lx = 0.1   (중심 → 앞/뒤 바퀴)
  ly = 0.065 (중심 → 좌/우 바퀴)
  R  = 0.045 (바퀴 반경)
"""

LX = 0.1
LY = 0.065
WHEEL_RADIUS = 0.045

# 바퀴 순서는 이 모듈 전체에서 고정: (FL, FR, RL, RR)
WHEEL_ORDER = ('front_left', 'front_right', 'rear_left', 'rear_right')


def inverse_kinematics(vx, vy, wz, lx=LX, ly=LY, r=WHEEL_RADIUS):
    """몸체 속도 (vx, vy, wz) -> 바퀴 각속도 (FL, FR, RL, RR) [rad/s]."""
    s = lx + ly
    fl = (vx - vy - s * wz) / r
    fr = (vx + vy + s * wz) / r
    rl = (vx + vy - s * wz) / r
    rr = (vx - vy + s * wz) / r
    return fl, fr, rl, rr


def forward_kinematics(fl, fr, rl, rr, lx=LX, ly=LY, r=WHEEL_RADIUS):
    """바퀴 각속도 (FL, FR, RL, RR) -> 몸체 속도 (vx, vy, wz).

    inverse_kinematics 의 정확한 역변환 (최소제곱해). 공식 mecanum_drive_controller 의
    odometry.cpp 와 같은 식이다. 여기서는 odom 이 아니라 '바퀴가 명령대로 굴렀다면 이만큼
    갔어야 한다'는 디버그 값(wheel_twist_debug)에만 쓴다.
    """
    s = lx + ly
    vx = r / 4.0 * (fl + fr + rl + rr)
    vy = r / 4.0 * (-fl + fr + rl - rr)
    wz = r / (4.0 * s) * (-fl + fr - rl + rr)
    return vx, vy, wz


def clamp(value, limit):
    """|value| <= limit 로 자름. limit <= 0 이면 제한 없음."""
    if limit is None or limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def scale_to_wheel_limit(wheels, max_wheel_speed):
    """가장 빠른 바퀴가 max_wheel_speed 를 넘으면 네 바퀴를 '같은 비율로' 줄인다.

    메카넘은 네 바퀴 속도의 '비율'이 이동 방향을 결정한다. 바퀴마다 따로 잘라내면
    (Webots 모터가 maxVelocity 초과 시 하는 일) 방향이 틀어지므로, 비율을 보존해 줄인다.
    반환: (조정된 바퀴 속도 튜플, 적용된 배율 <= 1.0)
    """
    peak = max(abs(w) for w in wheels)
    if max_wheel_speed <= 0.0 or peak <= max_wheel_speed:
        return tuple(wheels), 1.0
    k = max_wheel_speed / peak
    return tuple(w * k for w in wheels), k
