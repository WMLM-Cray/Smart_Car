# =============================================================================
# 阶段3 第三步:载入已存地图 + AMCL 定位一键启动(trackA + 车辆,无 SLAM)
# 用法(WSL,已 source ROS2 与工作区):
#   ros2 launch racecar_bringup_sim localization_bringup.launch.py \
#       [use_gui:=false] [world:=trackA.world] \
#       [spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0] \
#       [map:=<yaml>] [rviz:=true]
# 说明(与任务书第三步逐条对应):
#   - 重启仿真场景:复用 racecar_sim/sim_bringup.launch.py(车/控制器/接线箱);
#   - 关闭 SLAM 与测试驾驶器:本启动链不含 slam_toolbox、不含 teleop/手柄/
#     Nav2 导航栈,也不含任何速度发布者(运动由键盘/验收脚本另行发布);
#   - 加载保存地图:nav2 map_server(默认载入包内 maps/trackA_map_full,
#     即第二步整圈建图 map_saver 的正式地图);
#   - AMCL 定位:nav2 amcl(config/amcl.yaml;set_initial_pose 先给出生点
#     初始位姿,等效"人工设置初始位姿";交互时也可用 RViz 2D Pose Estimate
#     发 /initialpose 重给);
#   - 生命周期:nav2_lifecycle_manager 自动激活 map_server→amcl。
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('racecar_bringup_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racecar_sim'), 'launch',
            'sim_bringup.launch.py')),
        launch_arguments={
            'use_gui': LaunchConfiguration('use_gui'),
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl.yaml'),
                    {'yaml_filename': LaunchConfiguration('map')}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl.yaml')],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl.yaml')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'rviz',
                                      'localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_gui', default_value='false',
            description='gzclient 图形界面(定位演示关,以 RViz 为主屏)'),
        DeclareLaunchArgument('world', default_value='trackA.world'),
        DeclareLaunchArgument('spawn_x', default_value='11.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.75'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_share, 'maps',
                                       'trackA_map_full.yaml'),
            description='已存地图 yaml(默认第二步整圈地图)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='RViz 可视化(Xvfb/无头环境可置 false 仅定位)'),
        sim_bringup,
        map_server,
        amcl,
        lifecycle,
        rviz,
    ])
