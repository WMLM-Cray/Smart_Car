"""signal_exit_test：独立进程验证「SIGINT / SIGTERM 退出时，服务端是否确实结束在途目标」。

不接真车/真 Nav2：用 mock_nav2 子进程充当 NavigateToPose 服务器（behavior=never，
永不主动完成，直到被取消），用 mission_executor 子进程当任务器，向任务器进程发
SIGINT / SIGTERM，观察：
1) 服务端（mock）是否收到取消请求并置为 CANCELED（events_file 落盘）；
2) 任务器报告里在途目标是否被记作 canceled（取消已确认）还是 unconfirmed（未确认）。

运行（WSL 内，需 source ROS2 Humble + 本包 install）：
  python3 -m racecar_mission.signal_exit_test --signal SIGINT
  python3 -m racecar_mission.signal_exit_test --signal SIGTERM

输出：一行 JSON（signal / mock_events / executor_state / goal_status / stop_reason），
并在 --out 指定的文件里落一份，供交付取证。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time


def _pkg_config(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        base = os.path.join(get_package_share_directory('racecar_mission'), 'config')
    except Exception:
        base = os.path.join(os.path.dirname(__file__), '..', 'config')
    return os.path.join(base, name)


def _pkg_bin(name):
    """定位已安装的 console script 绝对路径，避免经 ros2 run CLI 包装层发信号。

    ``ros2 run`` 是一个 CLI 包装进程，向其 PID 发 SIGINT/SIGTERM 不会透传给真正节点
    （节点由它 fork/exec 启动，信号可能落到包装层或被忽略），导致“信号测试”测不到
    节点退出行为。这里直接执行 install/<pkg>/lib/<pkg>/<name> 里的可执行脚本，
    PID 就是节点进程，信号直接命中节点。
    """
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('racecar_mission')
        return os.path.join(prefix, 'lib', 'racecar_mission', name)
    except Exception:
        return os.path.join(os.path.dirname(__file__), '..', '..', '..',
                            'install', 'racecar_mission', 'lib', 'racecar_mission', name)


def _run_cli(args, timeout=15.0):
    """运行一个 CLI 命令（如 ros2 service call），返回 (returncode, stdout, stderr)。"""
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout, p.stderr


def _wait_file_json(path, key, timeout=20.0):
    """轮询一个 JSON 文件直到出现包含 key 的事件，返回解析后的对象（超时返回 None）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(path):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if any(ev and ev[0] == key for ev in data):
                    return data
            except Exception:
                pass
        time.sleep(0.05)
    return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--signal', choices=['SIGINT', 'SIGTERM'], required=True)
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    sig = getattr(signal, args.signal)
    tmp = tempfile.mkdtemp(prefix='racecar_mission_signal_')
    events_file = os.path.join(tmp, 'mock_events.json')
    report_file = os.path.join(tmp, 'report.json')
    goal_file = _pkg_config('mission_3point.yaml')

    mock = subprocess.Popen(
        [_pkg_bin('mock_nav2'),
         '--ros-args', '-p', 'behavior:=never',
         '-p', 'localization_topic:=/localization_ok',
         '-p', 'localization_rate:=20.0',
         '-p', 'events_file:=' + events_file],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    ex = subprocess.Popen(
        [_pkg_bin('mission_executor'),
         '--ros-args', '-p', 'goal_file:=' + goal_file,
         '-p', 'report_file:=' + report_file,
         '-p', 'localization_status_topic:=/localization_ok',
         '-p', 'cancel_timeout:=2.0',
         '-p', 'shutdown_wait_timeout:=2.0'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def _kill(p):
        if p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=3.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
                try:
                    p.wait(timeout=3.0)
                except Exception:
                    pass

    try:
        # 等 start 服务就绪后启动任务
        deadline = time.time() + 20.0
        started = False
        while time.time() < deadline and not started:
            rc, out, err = _run_cli(
                ['ros2', 'service', 'call', '/mission_executor/start',
                 'std_srvs/srv/Trigger', '{}'],
                timeout=10.0)
            started = (rc == 0 and 'success=True' in out)
            if not started:
                time.sleep(0.2)

        # 等在途目标已被服务端接受（events_file 出现 accepted）
        events = _wait_file_json(events_file, 'accepted', timeout=20.0) or []

        # 发信号让任务器退出
        try:
            ex.send_signal(sig)
        except Exception:
            pass

        # 等任务器进程退出
        try:
            ex.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            ex.kill()
            ex.wait()

        # 再给点时间让 mock 把取消事件落盘，然后终止 mock
        time.sleep(0.5)
        events = _wait_file_json(events_file, 'cancel_received', timeout=3.0) or events

        report = {}
        if os.path.exists(report_file):
            # 报告原子落盘；这里再兜底重试解析，防止读到未写完的空文件。
            read_deadline = time.time() + 3.0
            while time.time() < read_deadline:
                try:
                    with open(report_file, 'r', encoding='utf-8') as f:
                        report = json.load(f)
                    break
                except Exception:
                    time.sleep(0.05)

        goal_statuses = [g.get('status') for g in report.get('goals', [])]
        result = {
            'signal': args.signal,
            'mock_events': [list(e) for e in events] if events else [],
            'executor_exit_code': ex.returncode,
            'mission_state': report.get('mission', {}).get('state'),
            'stop_reason': report.get('mission', {}).get('stop_reason'),
            'goal_statuses': goal_statuses,
        }
        print(json.dumps(result, ensure_ascii=False))
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        return 0
    finally:
        # 无论成功/异常/超时，都确保两个子进程退出，绝不留孤儿进程污染后续回归。
        _kill(ex)
        _kill(mock)


if __name__ == '__main__':
    sys.exit(main())
