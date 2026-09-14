# =============================================================================
# 仿真一键启动:Gazebo + 车模 spawn + ros2_control 控制器激活
# 用法: ros2 launch racecar_sim sim_bringup.launch.py
# 验收链: 构建成功 → 控制器激活 → 直行/转向 → 传感器与TF → 一键启动
# =============================================================================
import os

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('racecar_sim')
    use_gui = LaunchConfiguration('use_gui')

    # 0) 控制器参数渲染:ros2_control.yaml.in 模板 + params.yaml 几何值
    #    → /tmp/racecar_ros2_control.yaml(独立验收问题4:几何参数原在
    #    ros2_control.yaml 与 params.yaml 双处维护,改一处另一处不跟;
    #    现在改 wheel_base/wheel_track/wheel_radius 只动 params.yaml,
    #    xacro 车模与控制器参数同源)。launch 阶段(任何进程启动前)写好,
    #    gzserver 内 controller_manager 与 spawner 都读这份。
    params = yaml.safe_load(
        open(os.path.join(pkg_share, 'config', 'params.yaml')))['racecar_sim']
    ctrl_tpl = open(os.path.join(
        pkg_share, 'config', 'ros2_control.yaml.in')).read()
    ctrl_params_path = '/tmp/racecar_ros2_control.yaml'
    ctrl_tpl = (ctrl_tpl
                .replace('__WHEELBASE__', str(float(params['wheel_base'])))
                .replace('__WHEEL_TRACK__', str(float(params['wheel_track'])))
                .replace('__WHEEL_RADIUS__', str(float(params['wheel_radius']))))
    open(ctrl_params_path, 'w').write(ctrl_tpl)

    # 1) robot_description:官方 demo 姿势(Python xacro + doc.toxml())。
    #    另:必须剥掉全部 XML 注释再传参。gazebo_ros2_control 0.4.10 源码
    #    (gazebo_ros2_control_plugin.cpp:223)会把本参数值拼成
    #    "robot_description:=<值>" 走 rcl_parse_arguments 注入,值部分按
    #    YAML 规则解析;XML 注释里的 "冒号+空格"/行尾冒号会被 YAML 当
    #    mapping 结构导致解析失败,插件 return 死亡、controller_manager
    #    永远不建(2026-09-14 读 0.4.10 源码+对照 demo 定位;demo 的
    #    <!-- Base Link --> 短注释无冒号故不炸,<?xml 声明行也非雷)
    xacro_file = os.path.join(pkg_share, 'urdf', 'racecar.xacro')
    doc = xacro.parse(open(xacro_file))
    # mappings:xacro 里 plugin 块 $(arg ctrl_params) 指向渲染产物
    xacro.process_doc(doc, mappings={'ctrl_params': ctrl_params_path})

    def strip_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == child.COMMENT_NODE:
                node.removeChild(child)
            else:
                strip_comments(child)
    strip_comments(doc)
    robot_xml = doc.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
        output='screen',
    )

    # 2) Gazebo Classic:系统二进制 gzserver/gzclient(Humble 的 gazebo_ros
    #    libexec 里没有 gzserver 包装可执行,Node 方式报 not found)。
    #    注意:gzserver 不解析 --ros-args(官方 gzserver.launch.py 无此透传,
    #    实测 gzserver 会把 remap 串当 world 路径打开报错)。
    #    /cmd_vel 接线与 odom TF 桥由 launch 末尾的 sim_wiring 节点完成。
    #    world 用自带 racecar.world(=empty.world + gazebo_ros_state 插件):
    #    libgazebo_ros_state.so 是 WorldPlugin,-s 只装 SystemPlugin,强装
    #    报 "incorrect plugin type" 且 factory 服务起不来(实测)。它提供
    #    /gazebo/get_entity_state,验收用世界系真实位姿与里程计对账——
    #    只看账面会被错误运动学骗过(2026-09-14 独立验收教训)
    world_file = os.path.join(pkg_share, 'worlds', 'racecar.world')
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        condition=IfCondition(use_gui),
        output='screen',
    )

    # 3) 车模 spawn(z 略抬防穿模,落地稳定)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'racecar',
                   '-z', '0.02'],
        output='screen',
    )

    # 4) 控制器激活(spawn 完成后依次拉起)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', ctrl_params_path],
        output='screen',
    )

    # 5) 接线箱:控制器激活后启动(/cmd_vel 转发 + odom TF 桥,见 sim_wiring.py)
    #    use_sim_time:全节点统一走 /clock 仿真时钟(复验补齐;该节点只透传
    #    消息不产生时间戳,但时钟一致性要求从配置上满足)
    sim_wiring = Node(
        package='racecar_sim',
        executable='sim_wiring',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 事件链:每步检查退出码,前一步失败就 Shutdown 整个 launch 并说明原因
    # (OnProcessExit 的 on_exit 传 callable(event, context) → 返回 action 列表;
    #  默认行为不分成败,spawn 失败也会继续拉起 spawner 干等,2026-09-14 复验指出)
    def _chain_on_ok(step_label, next_action):
        def _handler(event, context):
            rc = getattr(event, 'returncode', 0)
            if rc != 0:
                return [EmitEvent(event=Shutdown(
                    reason=f'{step_label} 失败(exit={rc}),终止启动链'))]
            return [next_action]
        return _handler

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='启动 gzclient 图形界面(headless 测试传 false)'),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity,
                          on_exit=_chain_on_ok('spawn_entity', jsb_spawner))
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner,
                          on_exit=_chain_on_ok('joint_state_broadcaster spawner', ackermann_spawner))
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=ackermann_spawner,
                          on_exit=_chain_on_ok('ackermann spawner', sim_wiring))
        ),
    ])
