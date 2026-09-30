#!/usr/bin/env python3
"""LIMO 메카넘(전방향) 구동 플러그인 — webots_ros2_driver 파이썬 플러그인.

기존 구조에서 'diff_drive_controller(ros2_control) + webots_ros2_control' 두 층이 하던 일을
이 파일 하나가 대신한다. 실물 LIMO 에서 cmd_vel 을 받아 IK 를 돌리고 모터를 구동하는
펌웨어(MODE_MCNAMU)와 같은 위치의 층이다.

  /{ns}/cmd_vel (Twist: vx, vy, wz)
      -> 속도 제한 (펌웨어처럼 vx / vy / wz 상한)
      -> 메카넘 역기구학 (mecanum_kinematics.inverse_kinematics)
      -> 바퀴 속도 비율 보존 포화 (모터 maxVelocity)
      -> RotationalMotor x4 setVelocity

diff_drive_controller 가 몰래 해주던 일도 여기서 직접 한다.
  - 명령 타임아웃: cmdTimeout 초 동안 cmd_vel 이 없으면 정지 (Nav2 가 죽어도 폭주 방지)
  - /{ns}/joint_states 발행: robot_state_publisher 가 바퀴 링크 TF 를 만들 수 있게
    (기존 joint_state_broadcaster 대체)

추가 (디버그 전용, Nav2 는 쓰지 않음):
  - /{ns}/wheel_twist_debug (TwistStamped): 실제 바퀴 회전량을 순기구학으로 바꾼 속도.
    gt_odom 의 진실 속도와 비교하면 비대칭 마찰 물리가 얼마나 미끄러지는지(측면 효율)를
    수치로 볼 수 있다. 나중에 바퀴 odom 으로 전환할 때도 이 계산을 그대로 쓴다.

URDF 파라미터 (limo.urdf 의 <plugin type="mecanum_driver.MecanumDriver"> 하위 태그):
  maxVx, maxVy, maxWz   [m/s, m/s, rad/s]  명령 상한 (0 이하면 제한 없음)
  cmdTimeout            [s]                명령 타임아웃
  jointStateRate        [Hz]               joint_states / 디버그 발행 주기
"""

import math

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState

from mecanum_kinematics import (WHEEL_ORDER, clamp, forward_kinematics,
                                inverse_kinematics, scale_to_wheel_limit)

# Webots 장치 이름 = URDF 조인트 이름 (LimoMecanum.proto / limo.urdf 와 동일해야 함)
MOTOR_NAMES = tuple(f'{w}_wheel' for w in WHEEL_ORDER)            # front_left_wheel ...
SENSOR_NAMES = tuple(f'{w}_wheel_sensor' for w in WHEEL_ORDER)    # front_left_wheel_sensor ...


def _prop(properties, key, default):
    """URDF <plugin> 하위 태그 값(문자열)을 float 로. 없으면 기본값."""
    try:
        return float(properties.get(key, default))
    except (TypeError, ValueError):
        return float(default)


class MecanumDriver:
    """webots_ros2_driver 플러그인 진입점 (init/step)."""

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()

        self.max_vx = _prop(properties, 'maxVx', 0.5)
        self.max_vy = _prop(properties, 'maxVy', 0.3)
        self.max_wz = _prop(properties, 'maxWz', 1.5)
        self.cmd_timeout = _prop(properties, 'cmdTimeout', 0.5)
        rate = _prop(properties, 'jointStateRate', 50.0)
        self.pub_period = 1.0 / rate if rate > 0.0 else 0.02

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('mecanum_driver', namespace=self.robot_name)
        log = self.node.get_logger()

        # --- 모터: 속도 제어 모드 (position=inf) ---
        self.motors = []
        for name in MOTOR_NAMES:
            m = self.robot.getDevice(name)
            if m is None:
                raise RuntimeError(f'[{self.robot_name}] 모터 "{name}" 없음 (proto 장치 이름 확인)')
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)
        # 모터가 낼 수 있는 최대 각속도. 넘으면 Webots 가 바퀴마다 따로 잘라 방향이 틀어지므로
        # scale_to_wheel_limit 로 비율을 보존해 미리 줄인다.
        self.max_wheel_speed = min(m.getMaxVelocity() for m in self.motors)

        # --- 엔코더 (joint_states, 디버그 속도용) ---
        self.sensors = []
        for name in SENSOR_NAMES:
            s = self.robot.getDevice(name)
            if s is None:
                raise RuntimeError(f'[{self.robot_name}] 센서 "{name}" 없음 (proto 장치 이름 확인)')
            s.enable(self.timestep)
            self.sensors.append(s)
        self.prev_pos = None
        self.prev_pos_time = None

        # --- 입출력 ---
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = None        # None = 아직 명령 없음 -> 정지 상태 유지
        self.timed_out = True
        self.node.create_subscription(Twist, f'/{self.robot_name}/cmd_vel', self._on_cmd, 10)
        self.js_pub = self.node.create_publisher(JointState, f'/{self.robot_name}/joint_states', 10)
        self.dbg_pub = self.node.create_publisher(
            TwistStamped, f'/{self.robot_name}/wheel_twist_debug', 10)
        self.last_pub_time = -1.0
        self.last_scale_warn = -1e9     # 첫 발생은 바로 경고

        log.info(
            f'[{self.robot_name}] mecanum_driver init OK '
            f'(max vx={self.max_vx} vy={self.max_vy} wz={self.max_wz}, '
            f'timeout={self.cmd_timeout}s, motor max={self.max_wheel_speed:.2f} rad/s)')

    # ------------------------------------------------------------------
    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.robot.getTime()

    def _stamp(self, t):
        return RosTime(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    # ------------------------------------------------------------------
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()

        # --- 1) 명령 선택: 타임아웃이면 정지 ---
        fresh = self.last_cmd_time is not None and (t - self.last_cmd_time) <= self.cmd_timeout
        if fresh:
            vx, vy, wz = self.cmd
            self.timed_out = False
        else:
            if not self.timed_out and self.last_cmd_time is not None:
                self.node.get_logger().warn(
                    f'[{self.robot_name}] cmd_vel {self.cmd_timeout}s 동안 없음 -> 정지')
            self.timed_out = True
            vx = vy = wz = 0.0

        # --- 2) 몸체 속도 상한 (실물 펌웨어의 속도 제한 흉내) ---
        vx = clamp(vx, self.max_vx)
        vy = clamp(vy, self.max_vy)
        wz = clamp(wz, self.max_wz)

        # --- 3) 역기구학 -> 4) 비율 보존 포화 -> 5) 모터 인가 ---
        wheels = inverse_kinematics(vx, vy, wz)
        wheels, k = scale_to_wheel_limit(wheels, self.max_wheel_speed)
        if k < 1.0 and t - self.last_scale_warn > 2.0:
            self.node.get_logger().warn(
                f'[{self.robot_name}] 바퀴 속도가 모터 한계 초과 -> 전체 {k:.2f}배로 축소 '
                f'(방향 보존)')
            self.last_scale_warn = t
        for motor, w in zip(self.motors, wheels):
            motor.setVelocity(w)

        # --- 6) joint_states + 디버그 속도 (jointStateRate 주기) ---
        if t - self.last_pub_time >= self.pub_period:
            self._publish_states(t)
            self.last_pub_time = t

    # ------------------------------------------------------------------
    def _publish_states(self, t):
        pos = [s.getValue() for s in self.sensors]
        if any(math.isnan(p) for p in pos):     # 첫 스텝 등 센서값 미준비
            return
        stamp = self._stamp(t)

        vel = [0.0, 0.0, 0.0, 0.0]
        if self.prev_pos is not None and t > self.prev_pos_time:
            dt = t - self.prev_pos_time
            vel = [(p - q) / dt for p, q in zip(pos, self.prev_pos)]
        self.prev_pos, self.prev_pos_time = pos, t

        js = JointState()
        js.header.stamp = stamp
        js.name = list(MOTOR_NAMES)
        js.position = pos
        js.velocity = vel
        self.js_pub.publish(js)

        # 실제 바퀴 회전 -> 순기구학 속도 (base_link 기준). Nav2 는 쓰지 않는 디버그 값.
        dvx, dvy, dwz = forward_kinematics(*vel)
        dbg = TwistStamped()
        dbg.header.stamp = stamp
        dbg.header.frame_id = f'{self.robot_name}/base_link'
        dbg.twist.linear.x = dvx
        dbg.twist.linear.y = dvy
        dbg.twist.angular.z = dwz
        self.dbg_pub.publish(dbg)
