"""racecar_mission: 多点导航任务器。

纯 Python 核心（goal/validate/engine/report）不依赖 rclpy，可在无 ROS 环境下做
单元测试；executor_node / mock_nav2 / offline_regression 需要 ROS2 Humble。
"""
__version__ = '0.1.0'
