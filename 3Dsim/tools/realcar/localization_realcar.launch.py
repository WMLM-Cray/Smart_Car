# 实车 AMCL 定位链 (阶段2, 2026-09-17)
# 只含 map_server + amcl + lifecycle_manager 三节点。
# 刻意不含: planner_server / controller_server / bt_navigator / costmap /
#           slam_toolbox / waypoint_cycle / 任何速度发布器 (任务书启动边界)
# 地图与 AMCL 参数从命令行传入, 不写死房间坐标。
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='map_server YAML 绝对路径 (如 ~/racecar_team/maps/room_20260917/map.yaml)'),
        DeclareLaunchArgument(
            'amcl_params', description='AMCL 参数 YAML 绝对路径 (如 ~/racecar_team/tools/amcl/amcl_realcar.yaml)'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': LaunchConfiguration('map'),
            }],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_params')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])
