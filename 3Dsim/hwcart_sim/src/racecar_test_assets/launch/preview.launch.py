"""Standalone preview launch: open a racecar_test_assets world in Gazebo Classic.

No vehicle is spawned by design - the main project's bringup is responsible
for that. Paths resolve relative to this file, so it works both from a
source checkout and from the installed (colcon) package share directory.

Usage:
  ros2 launch racecar_test_assets preview.launch.py                          # sensor_test.world, server+client
  ros2 launch racecar_test_assets preview.launch.py world:=demo_world.world   # demo scene (all five model families)
  ros2 launch racecar_test_assets preview.launch.py gui:=false                # headless gzserver
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

PKG_ROOT = Path(__file__).resolve().parent.parent


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    world_arg = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([str(PKG_ROOT / 'worlds'), world_arg])
    models = str(PKG_ROOT / 'models')

    env = dict(os.environ)
    existing = env.get('GAZEBO_MODEL_PATH', '')
    env['GAZEBO_MODEL_PATH'] = models + ((':' + existing) if existing else '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='true: gazebo (server+client); false: gzserver only'),
        DeclareLaunchArgument(
            'world', default_value='sensor_test.world',
            description='world file in worlds/ (sensor_test.world or demo_world.world)'),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path], env=env, output='screen',
            condition=IfCondition(gui)),
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path], env=env, output='screen',
            condition=UnlessCondition(gui)),
    ])
