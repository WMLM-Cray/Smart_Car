"""运行实际 ROS 导入前初始化，捕获纯 AST 函数测试覆盖不到的启动错误。"""
from pathlib import Path

def test_web_initialization_before_ros_import():
    path = Path(__file__).with_name('server.py')
    source = path.read_text(encoding='utf-8').split('import rclpy', 1)[0]
    scope = {'__file__': str(path), '__name__': 'startup_probe'}
    exec(compile(source, str(path), 'exec'), scope)
    assert scope['CarParams']().pwm_mid == 1540
