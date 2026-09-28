"""验收脚本的参数校验:传了节点不认的环境变量,必须当场报错而不是静默吞掉。

这两个脚本的坑是同一类,而且都**不会**以"脚本挂了"的形式表现出来:

  * run_road_policy_once.sh 曾按老分支的约定给 v2 节点传 --sim / --guard /
    --scan-timeout,而 dreamfish 的 road_policy_node.py 三个都不认(argparse
    直接 exit 2)。节点是后台进程,脚本看不见它死了,会照跑 120s 然后产出一份
    laps=0 的完整 JSON —— 和"模型跑不动"长得一模一样。
  * run_s3_policy_once.sh 曾给 v1 节点传 --scan-timeout。那个节点手写解析参数,
    未知选项不报错,但它的值会漏进位置参数里,SCAN_TIMEOUT 静默失效。

所以现在的约定是:传不进去就 exit 5,宁可没有数据也不要假数据。这些测试就是
把这个约定钉住 —— 它们只跑参数校验分支(在起 gzserver 之前就退出),不需要 ROS。
"""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

# 用 PATH 上的 bash 的完整路径:Windows 上裸写 'bash' 会先命中 System32 下的 bash.exe
# (WSL 启动器),它看不到 Windows 路径,脚本直接 127;shutil.which 走 PATH,拿到的是 Git Bash
BASH = shutil.which('bash')
pytestmark = pytest.mark.skipif(BASH is None, reason='没有 bash')

ROOT = Path(__file__).resolve().parents[2]
GAZEBO = ROOT / 'simulation' / 'gazebo'
WORLD = GAZEBO / 'src' / 'racecar_sim' / 'worlds' / 'trackA.world'
ROAD = GAZEBO / 'run_road_policy_once.sh'
S3 = GAZEBO / 'run_s3_policy_once.sh'


def run(script, env=None, extra=None):
    """跑一次脚本,只走到参数校验。用不存在的 layout/输出路径无所谓 —— 校验在
    读它们之前就退出了,能走到那一步的用例这里不测(那需要 Gazebo)。"""
    e = dict(os.environ)
    for key in ('GUARD', 'SCAN_TIMEOUT', 'POLICY_SPEED', 'WORKER'):
        e.pop(key, None)
    e.update(env or {})
    cmd = [BASH, str(script), str(WORLD), 'models/road_v2/candidate_167.npz',
           '/tmp/_never.yaml', '/tmp/_never.json'] + (extra or [])
    return subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace', env=e, timeout=60)


def test_world_without_state_plugin_is_rejected(tmp_path):
    """没带 gazebo_ros_state 的 world 拿不到 /model_states,打分全空,必须先挡。"""
    bare = tmp_path / 'bare.world'
    bare.write_text('<sdf version="1.6"><world name="w"/></sdf>', encoding='utf-8')
    r = subprocess.run([BASH, str(S3), str(bare), 'm.npz', '/tmp/x.yaml', '/tmp/x.json'],
                       capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=60)
    assert r.returncode == 2
    assert 'gazebo_ros_state' in r.stderr


@pytest.mark.parametrize('script', [ROAD, S3])
def test_missing_args_is_usage_error(script):
    r = subprocess.run([BASH, str(script)], capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=60)
    assert r.returncode == 2
    assert 'usage' in r.stderr


def test_road_script_rejects_guard_because_v2_has_no_guard():
    """v2 仿真节点没有 ScanGuard,GUARD 无处可传。吞掉它会让撞锥统计悄悄从
    guard=0.25 口径变成纯物理碰撞,和 v1 的验收不可比。"""
    r = run(ROAD, {'GUARD': '0.25'})
    assert r.returncode == 5, r.stderr
    assert 'GUARD=0.25' in r.stderr
    assert '没有防撞' in r.stderr


def test_s3_script_rejects_scan_timeout_because_v1_node_ignores_it():
    r = run(S3, {'SCAN_TIMEOUT': '0.3'})
    assert r.returncode == 5, r.stderr
    assert 'SCAN_TIMEOUT=0.3' in r.stderr


def test_guard_check_happens_before_gzserver_starts():
    """参数错误不该等 gzserver 起完(一分多钟)才报。退出码 5 是参数校验,
    不是 3(端口占用)/4(仿真中途死)/6(策略节点已退出)。"""
    assert run(ROAD, {'GUARD': '0.25'}).returncode == 5
    assert run(S3, {'SCAN_TIMEOUT': '0.3'}).returncode == 5


def _launch_line(script_text, node_file):
    """取出真正启动策略节点的那条命令 —— 从 `python3 .../<node_file>` 开始,
    拼到行尾的 `&` 为止。注释里也会提到节点文件名,不能按文件名整行匹配。"""
    lines = script_text.splitlines()
    for i, ln in enumerate(lines):
        if ln.lstrip().startswith('python3') and node_file in ln:
            cmd, j = ln, i
            while cmd.rstrip().endswith('\\'):      # 续行拼回去
                j += 1
                cmd = cmd.rstrip()[:-1] + ' ' + lines[j]
            return cmd
    raise AssertionError(f'找不到启动 {node_file} 的那一行')


def test_scripts_only_pass_flags_their_node_accepts():
    """钉住命令行本身。老分支的约定是 --sim/--speed/--guard/--scan-timeout,
    dreamfish 的节点不通用 —— 改回去就等于让车一步不动地空跑。"""
    road = _launch_line(ROAD.read_text(encoding='utf-8'), 'road_policy_node.py')
    assert '--speed-cap' in road
    # --speed 只作为 argparse 前缀缩写才"能用",节点加个 --speed-xxx 就歧义。
    assert '--sim' not in road
    assert '--guard' not in road
    assert '--scan-timeout' not in road

    s3 = _launch_line(S3.read_text(encoding='utf-8'), 'steer_policy_node.py')
    assert '--scan-timeout' not in s3
    assert '--sim' in s3            # v1 节点确实认这个
    assert '--speed' in s3
