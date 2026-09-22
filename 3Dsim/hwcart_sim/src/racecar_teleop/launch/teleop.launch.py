"""Launch file for racecar_teleop.

Two modes, selected by the ``use_joy`` launch argument:

  * ``use_joy:=true``  (default) — also launch the ``joy`` driver with an
    explicit ``autorepeat_rate``.  Humble's joy 3.3.0 already defaults to
    20.0, but 0.0 would mean publish-only-on-change (the ROS 1 joy and
    joy_linux default), which would trip the 0.3 s input timeout while
    driving with steady inputs — so pin it instead of relying on the
    driver/version default.
  * ``use_joy:=false`` — launch ONLY the mapping node, so a separate simulated
    ``/joy`` publisher can feed it (avoids two publishers on the same topic).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = 'racecar_teleop'
    config = os.path.join(
        get_package_share_directory(pkg), 'config', 'generic.yaml')
    use_joy = LaunchConfiguration('use_joy')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy', default_value='true',
            description='Launch the joy driver node in addition to teleop'),
        DeclareLaunchArgument(
            'joy_autorepeat_rate', default_value='20.0',
            description='joy_node autorepeat rate in Hz. joy (Humble, 3.3.0) '
                        'already defaults to 20.0 and republishes only on '
                        'change when set to 0.0 (the ROS 1 joy / joy_linux '
                        'default), which violates the <0.3 s /joy gap this '
                        'package requires — hence pinned explicitly. Keep '
                        'well above 1/input_timeout (~3.3 Hz); joy caps it '
                        'at 1000.0.'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='screen',
            # Pin autorepeat_rate explicitly (see joy_autorepeat_rate above)
            # so the <0.3 s /joy gap never depends on driver/version defaults.
            parameters=[{
                'autorepeat_rate': ParameterValue(
                    LaunchConfiguration('joy_autorepeat_rate'), value_type=float),
            }],
            condition=IfCondition(use_joy),
        ),
        Node(
            package=pkg, executable='teleop_node', name='teleop_node',
            output='screen',
            parameters=[config],
            # Defaults already match /joy and /teleop/cmd_vel; kept explicit so
            # the main project can remap here without touching the code.
            remappings=[
                ('/joy', '/joy'),
                ('/teleop/cmd_vel', '/teleop/cmd_vel'),
            ],
        ),
    ])
