"""LIMO 단독 Webots 스택 launch (Supervisor + 드라이버 + ros2_control + Nav2).

Ros2Supervisor 도 여기서 함께 띄운다 (Supervisor 가 없으면 /clock 이 나오지 않아 시뮬이 진행되지 않는다).

사용:
  webots ~/LIMO_webots/worlds/limo_10x10.wbt          # 먼저 월드
  source ~/LIMO_webots/setup_env.sh
  ros2 launch ~/LIMO_webots/limo.launch.py             # robots:=limo (월드의 로봇 name 과 일치)

Nav2 목표 보내기:
  ros2 action send_goal /limo/navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
"""
import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

ROOT = os.path.dirname(os.path.abspath(__file__))
URDF_TPL = os.path.join(ROOT, 'controllers', 'limo.urdf')        # __NS__ 자리표시자
ROS2CONTROL = os.path.join(ROOT, 'config', 'limo_ros2control.yaml')
NAV2_SRC = os.path.join(ROOT, 'config', 'limo_nav2_params.yaml')


def make_urdf(ns):
    """템플릿의 __NS__ 를 로봇 이름으로 치환해 /tmp/{ns}.urdf 생성."""
    with open(URDF_TPL) as f:
        s = f.read()
    assert '__NS__' in s, 'limo.urdf 템플릿에 __NS__ 자리표시자가 없음'
    out = f'/tmp/{ns}.urdf'
    with open(out, 'w') as f:
        f.write(s.replace('__NS__', ns))
    return out


def make_nav2_params(ns):
    """Nav2 params 의 노드 키·프레임·토픽을 /{ns} 버전으로 치환해 /tmp 에 생성."""
    with open(NAV2_SRC) as f:
        s = f.read()
    for k in ['bt_navigator', 'controller_server', 'global_costmap',
              'local_costmap', 'planner_server', 'behavior_server']:
        s = re.sub(rf'(?m)^{k}:', f'/{ns}/{k}:', s)
    s = s.replace('robot_base_frame: base_link', f'robot_base_frame: {ns}/base_link')
    s = s.replace('global_frame: odom', f'global_frame: {ns}/odom')
    s = s.replace('odom_topic: /odom', f'odom_topic: /{ns}/odom')
    s = s.replace('topic: /scan', f'topic: /{ns}/scan')
    out = f'/tmp/{ns}_nav2_params.yaml'
    with open(out, 'w') as f:
        f.write(s)
    return out


def make_limo_stack(ns):
    urdf = make_urdf(ns)
    nav2_params = make_nav2_params(ns)
    with open(urdf) as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=ns, output='screen',
        parameters=[{'robot_description': robot_desc, 'frame_prefix': f'{ns}/', 'use_sim_time': True}])

    driver = WebotsController(
        robot_name=ns, namespace=ns,
        parameters=[
            {'robot_description': urdf, 'set_robot_state_publisher': False, 'use_sim_time': True},
            ROS2CONTROL,
        ],
        remappings=[
            # Humble: use_stamped_vel=false 이면 DiffDriveController 구독 이름이 ~/cmd_vel_unstamped
            (f'/{ns}/diffdrive_controller/cmd_vel_unstamped', f'/{ns}/cmd_vel'),
            (f'/{ns}/diffdrive_controller/odom', f'/{ns}/_unused_odom_encoder'),
            (f'/{ns}/laser', f'/{ns}/scan'),
        ],
        respawn=True)

    cm_timeout = ['--controller-manager-timeout', '50']
    jsb = Node(package='controller_manager', executable='spawner', namespace=ns, output='screen',
               arguments=['joint_state_broadcaster', '-c', f'/{ns}/controller_manager'] + cm_timeout)
    ddc = Node(package='controller_manager', executable='spawner', namespace=ns, output='screen',
               arguments=['diffdrive_controller', '-c', f'/{ns}/controller_manager'] + cm_timeout)
    waiting = WaitForControllerConnection(target_driver=driver, nodes_to_start=[jsb, ddc])

    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        namespace=ns, output='screen', parameters=[{'use_sim_time': True}],
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', f'{ns}/odom'])

    use_sim = {'use_sim_time': True}
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_planner', executable='planner_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_behaviors', executable='behavior_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=ns, output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'bt_navigator']}])
    nav2_delayed = TimerAction(period=12.0, actions=nav2_nodes + [lifecycle])

    return [rsp, driver, waiting, static_map_odom, nav2_delayed]


def launch_setup(context):
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    actions = []
    for ns in robots:
        actions += make_limo_stack(ns)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='limo',
                              description='쉼표 구분 LIMO 이름 목록 (월드의 로봇 name 과 일치)'),
        Ros2SupervisorLauncher(port='1234'),     # /clock 발행. 이게 있어야 시뮬이 진행된다
        OpaqueFunction(function=launch_setup),
    ])
