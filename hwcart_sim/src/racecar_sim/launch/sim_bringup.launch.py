# =============================================================================
# 仿真一键启动:Gazebo + 车模 spawn + ros2_control 控制器激活
# 用法: ros2 launch racecar_sim sim_bringup.launch.py
# 验收链: 构建成功 → 控制器激活 → 直行/转向 → 传感器与TF → 一键启动
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('racecar_sim')
    use_gui = LaunchConfiguration('use_gui')

    # 1) robot_description:xacro 实时展开(参数单一来源 params.yaml)
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('racecar_sim'), 'urdf', 'racecar.xacro'])
    ])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # ParameterValue 强制按字符串传,防 launch_ros 把 URDF 当 yaml 解析报错
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 2) Gazebo Classic:系统二进制 gzserver/gzclient(Humble 的 gazebo_ros
    #    libexec 里没有 gzserver 包装可执行,Node 方式报 not found)
    #    cmd_vel 重映射经 --ros-args 传入:libgazebo_ros_init.so 会把 gzserver
    #    的 argv 交给 rclcpp 初始化,进程内 controller_manager(即
    #    gazebo_ros2_control 插件)继承该重映射:控制器 ~/cmd_vel → /cmd_vel
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--ros-args', '-r',
             '/ackermann_steering_controller/cmd_vel:=/cmd_vel'],
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
                   '--param-file', os.path.join(pkg_share, 'config', 'ros2_control.yaml')],
        output='screen',
    )

    # 事件链:spawn 退出(成功) → joint_state_broadcaster → ackermann
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='启动 gzclient 图形界面(headless 测试传 false)'),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[jsb_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[ackermann_spawner])
        ),
    ])
