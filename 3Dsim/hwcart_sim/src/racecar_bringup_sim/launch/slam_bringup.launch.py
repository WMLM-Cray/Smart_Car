# =============================================================================
# 阶段3 第一步:二维建图演示一键启动(trackA + 车辆 + slam_toolbox + RViz)
# 用法(WSL,已 source ROS2 与工作区):
#   ros2 launch racecar_bringup_sim slam_bringup.launch.py \
#       [use_gui:=false] [world:=trackA.world] \
#       [spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0]
# 说明:
#   - 复用 racecar_sim/sim_bringup.launch.py(车辆/控制器/接线箱/事件链),
#     只追加 slam_toolbox(online_async)与 RViz(slam.rviz);
#   - slam_toolbox 默认参数与本项目链路零参数匹配(base_frame=base_footprint,
#     odom_frame=odom,scan_topic=/scan),显式传 use_sim_time;
#   - SLAM 只用模拟传感器 /scan + 正常 TF(odom→base_footprint 由 sim_wiring
#     桥接),不注入任何 Gazebo 真值定位(阶段3 任务书边界);
#   - RViz 显示:Map(/map)+LaserScan(/scan)+RobotModel+TF+Odometry 轨迹
#     (/ackermann_steering_controller/odometry);无显示环境(Xvfb/SSH)时
#     可用 env RVIZ_SCREEN=0 跳过 RViz 仅建图。
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.logging


def generate_launch_description():
    bringup_share = get_package_share_directory('racecar_bringup_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescription(os.path.join(
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

    slam = IncludeLaunchDescription(
        PythonLaunchDescription(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch',
            'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_gui', default_value='false',
            description='gzclient 图形界面(建图演示通常关,以 RViz 为主屏)'),
        DeclareLaunchArgument('world', default_value='trackA.world'),
        DeclareLaunchArgument('spawn_x', default_value='11.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.75'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='RViz 可视化(Xvfb/无头环境可置 false 仅建图)'),
        sim_bringup,
        slam,
        rviz,
    ])
