# =============================================================================
# 阶段3 第四步:Nav2 单点导航一键启动(trackA + 车 + 已存地图 + AMCL + Nav2)
# 用法(WSL,已 source ROS2 与工作区):
#   ros2 launch racecar_bringup_sim navigation_bringup.launch.py \
#       [use_gui:=false] [world:=trackA.world] \
#       [spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0] \
#       [map:=<yaml>] [nav_rviz:=true]
# 组成:
#   - localization_bringup(仿真+map_server+amcl+lifecycle 定位管理器,
#     rviz:=false,本链用 navigation.rviz 单 RViz);
#   - Nav2:planner_server(NavFn)/controller_server(RegulatedPurePursuit,
#     use_rotate_to_heading=false 不原地转向)/velocity_smoother(加加速度
#     平滑,输出唯一 /cmd_vel)/behavior_server(仅 Wait)/bt_navigator
#     (trackA_minimal.xml 恢复分支无 Spin/BackUp);
#   - lifecycle_manager_navigation 自动激活上述五节点(与定位管理器互不重叠)。
# 速度链:controller → cmd_vel_nav → velocity_smoother → /cmd_vel → sim_wiring → 车
# =============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('racecar_bringup_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- 定位链(仿真+map_server+AMCL):复用第三步,rviz:=false ----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'localization_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gui': LaunchConfiguration('use_gui'),
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'map': LaunchConfiguration('map'),
            'rviz': 'false',
        }.items(),
    )

    nav_params = os.path.join(bringup_share, 'config', 'nav2.yaml')
    bt_xml = os.path.join(bringup_share, 'behavior', 'trackA_minimal.xml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],   # 输出给速度平滑器
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav_params],
        remappings=[('cmd_vel', 'cmd_vel_nav'),        # 输入:控制器
                    ('cmd_vel_smoothed', 'cmd_vel')],  # 输出:唯一 /cmd_vel
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_params],
        # humble behavior_server 默认在 /cmd_vel 建发布者(即使只装 Wait 也
        # 从不发速度)→ 重映射挪开,保住 /cmd_vel 唯一发布者=velocity_smoother
        remappings=[('cmd_vel', 'behavior_cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params,
                    # humble 在 configure 时会校验两棵默认树(ToPose+ThroughPoses),
                    # 都指到极简树:本链只发 NavigateToPose 目标;ThroughPoses
                    # 动作未配置专用树(扩展时需补 RemovePassedGoals 等插件库)
                    {'default_bt_xml_filename': bt_xml,
                     'default_nav_to_pose_bt_xml': bt_xml,
                     'default_nav_through_poses_bt_xml': bt_xml}],
    )

    # 延迟 10s:避开 WSL 负载下的管理器/节点 configure 竞态(同定位链注释)
    lifecycle = TimerAction(
        period=10.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav_params],
        )])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'rviz',
                                      'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        # 参数名用 nav_rviz 而非 rviz:本链 include 定位链时必须传
        # rviz:=false 关掉子链 RViz,同名实参会泄漏进父作用域把本节点
        # 的 IfCondition 一并判 false(2026-09-16 对照实验定位)
        condition=IfCondition(LaunchConfiguration('nav_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_gui', default_value='false',
            description='gzclient(导航演示关,RViz 为主屏)'),
        DeclareLaunchArgument('world', default_value='trackA.world'),
        DeclareLaunchArgument('spawn_x', default_value='11.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.75'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup_share, 'maps',
                                              'trackA_map_full.yaml'),
            description='已存地图 yaml(第三步同款)'),
        DeclareLaunchArgument(
            'nav_rviz', default_value='true',
            description='RViz(含代价地图/路径显示,2D Goal Pose 发目标;'
                        '注:定位子链的 rviz 参数已被本链固定为 false)'),
        localization,
        controller_server,
        velocity_smoother,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle,
        rviz,
    ])
