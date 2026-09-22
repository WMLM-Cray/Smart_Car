"""use_sim_time_check：验证 launch 启动后 use_sim_time 参数确实生效。

回执要求「launch 启动后实际读取 use_sim_time，证明参数生效」。这里用真实
``ros2 launch racecar_mission executor.launch.py use_sim_time:=true`` 启动任务器，
再 ``ros2 param get /mission_executor use_sim_time`` 读回参数，证明 launch 声明 ->
参数表 -> rclpy 节点（clock 选择）这条链路确实把 true 传了进去，而非仅文档声明。

运行（WSL 内，需 source ROS2 Humble + 本包 install）：
  python3 -m racecar_mission.use_sim_time_check --goal <3point.yaml> --out <json>

输出：一行 JSON（use_sim_time / param_value / param_get_raw）。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time


def _pkg_config(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        base = os.path.join(get_package_share_directory('racecar_mission'), 'config')
    except Exception:
        base = os.path.join(os.path.dirname(__file__), '..', 'config')
    return os.path.join(base, name)


def _run(args, timeout=15.0):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout.strip(), p.stderr.strip()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', default='')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    goal = args.goal or _pkg_config('mission_3point.yaml')

    launch = subprocess.Popen(
        ['ros2', 'launch', 'racecar_mission', 'executor.launch.py',
         'goal_file:=' + goal, 'use_sim_time:=true'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    param_value = None
    param_get_raw = ''
    try:
        # 等 /mission_executor 节点参数可查询
        deadline = time.time() + 30.0
        while time.time() < deadline:
            rc, out, err = _run(
                ['ros2', 'param', 'get', '/mission_executor', 'use_sim_time'], timeout=8.0)
            if rc == 0 and out:
                param_get_raw = out
                low = out.lower()
                if 'true' in low:
                    param_value = True
                elif 'false' in low:
                    param_value = False
                break
            time.sleep(0.5)
    finally:
        # 终止 launch（SIGINT 会让 launch 优雅停止子节点）
        try:
            launch.send_signal(signal.SIGINT)
            launch.wait(timeout=10.0)
        except Exception:
            try:
                launch.kill()
                launch.wait(timeout=5.0)
            except Exception:
                pass

    result = {
        'use_sim_time': param_value,
        'param_get_raw': param_get_raw,
        'expected': True,
    }
    print(json.dumps(result, ensure_ascii=False))
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
    return 0 if param_value is True else 1


if __name__ == '__main__':
    sys.exit(main())
