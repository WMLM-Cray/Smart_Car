"""启动 mission_executor（多点导航任务器）。

用法：
  ros2 launch racecar_mission executor.launch.py \
    goal_file:=<绝对路径> [report_file:=...] [use_sim_time:=true]

任务器只调度 Nav2 NavigateToPose，不启动 Nav2 / AMCL / 地图（这些由主线提供）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('racecar_mission')

    goal_file = LaunchConfiguration('goal_file')
    report_file = LaunchConfiguration('report_file')
    checkpoint_file = LaunchConfiguration('checkpoint_file')
    resume_from = LaunchConfiguration('resume_from')
    nav_action = LaunchConfiguration('nav_action')
    loc_topic = LaunchConfiguration('localization_status_topic')
    loc_watchdog = LaunchConfiguration('localization_watchdog')
    use_sim_time = LaunchConfiguration('use_sim_time')

    node = Node(
        package='racecar_mission',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[{
            'goal_file': goal_file,
            'report_file': report_file,
            'checkpoint_file': checkpoint_file,
            'resume_from': resume_from,
            'nav_action': nav_action,
            'localization_status_topic': loc_topic,
            'localization_watchdog': loc_watchdog,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('goal_file', default_value='',
                              description='任务 YAML 绝对路径'),
        DeclareLaunchArgument('report_file', default_value='',
                              description='JSON 报告输出路径（空则 ./mission_report.json）'),
        DeclareLaunchArgument('checkpoint_file', default_value='',
                              description='检查点输出路径（空则不写检查点）'),
        DeclareLaunchArgument('resume_from', default_value='',
                              description='检查点输入路径（显式恢复，空则全新开始）'),
        DeclareLaunchArgument('nav_action', default_value='/navigate_to_pose',
                              description='Nav2 NavigateToPose 动作名'),
        DeclareLaunchArgument('localization_status_topic', default_value='/localization_ok',
                              description='定位健康 std_msgs/Bool 话题'),
        DeclareLaunchArgument('localization_watchdog', default_value='0.0',
                              description='定位无信号超时秒数（0=禁用 watchdog）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用仿真时间（与 Gazebo /rosbag play 一致）'),
        node,
    ])
