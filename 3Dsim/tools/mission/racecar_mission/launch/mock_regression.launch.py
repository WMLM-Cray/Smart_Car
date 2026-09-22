"""离线回归：mock_nav2 + mission_executor 一起启动。

用法（不依赖 Gazebo / Nav2，仅需 ROS2 Humble）：
  ros2 launch racecar_mission mock_regression.launch.py \
    goal_file:=<包内 config/mission_3point.yaml 的绝对路径> \
    mock_behavior:=success \
    report_file:=/tmp/mission_report.json

随后用 ros2 service call 驱动：
  ros2 service call /mission_executor/start std_srvs/srv/Trigger "{}"
  ros2 service call /mission_executor/status std_srvs/srv/Trigger "{}"
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
    mock_behavior = LaunchConfiguration('mock_behavior')
    mock_delay = LaunchConfiguration('mock_delay')
    use_sim_time = LaunchConfiguration('use_sim_time')

    mock = Node(
        package='racecar_mission',
        executable='mock_nav2',
        name='mock_nav2',
        output='screen',
        parameters=[{
            'action': '/navigate_to_pose',
            'behavior': mock_behavior,
            'delay': mock_delay,
            'localization_topic': '/localization_ok',
            'use_sim_time': use_sim_time,
        }],
    )

    executor = Node(
        package='racecar_mission',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[{
            'goal_file': goal_file,
            'report_file': report_file,
            'nav_action': '/navigate_to_pose',
            'nav_wait_timeout': 5.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('goal_file', default_value='',
                              description='任务 YAML 绝对路径'),
        DeclareLaunchArgument('report_file', default_value='',
                              description='JSON 报告输出路径'),
        DeclareLaunchArgument('mock_behavior', default_value='success',
                              description='mock 行为序列（success/abort/reject/never/die，逗号分隔）'),
        DeclareLaunchArgument('mock_delay', default_value='0.5',
                              description='mock 完成前等待秒数'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用仿真时间'),
        mock,
        executor,
    ])
