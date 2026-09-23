"""carready 的纯逻辑部分:挑残留进程、判体检结果。不需要 ROS。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carready import PWM_MID, SLEEP_UNITS, esc_link_ok, find_strays, problems  # noqa: E402

PS = [
    '  1 /sbin/init',
    '615 python3 /home/orange-pi/web_teleop/watchdog.py',
    '616 python3 /home/orange-pi/web_teleop/server.py',
    '1749 python3 steer_policy_node.py runs/candidate_060.npz --pwm 1610',
    '2001 python3 ramp_curve.py --top 1620',
    '2002 /usr/bin/python3 /home/orange-pi/autocar/esc_rearm.py --no-cycle',
    '2003 bash -c cd ~/autocar && python3 ramp_curve.py',      # 发起它的 shell,不能杀
    '2004 python3 /home/orange-pi/autocar/carready.py',
    '2005 vim ramp_curve.py',
]

GOOD = dict(
    units={'racecar-chassis': 'active', 'racecar-watchdog': 'active', 'racecar-web': 'active'},
    rates={'/encoder': 38.0, '/scan': 12.0, '/IMU_data': 55.0},
    cmd_pubs=['teleop_watchdog', 'web_teleop', 'steer_policy_node'],
    cmd_subs=['teleop_watchdog', 'racecar_driver'],
    scan_pubs=1, idle_cmds=0, esc_ok=True,
    sleep_states={u: 'masked' for u in SLEEP_UNITS})


def pids(strays):
    return sorted(p for p, _ in strays)


def test_picks_only_test_scripts_run_by_python():
    assert pids(find_strays(PS, self_pid=2004)) == [1749, 2001, 2002]


def test_never_matches_the_shell_that_launched_it():
    # pkill -f 的老坑:匹配到 ssh 命令行本身,把自己的会话杀了
    assert 2003 not in pids(find_strays(PS, self_pid=2004))


def test_leaves_services_and_editors_alone():
    got = pids(find_strays(PS, self_pid=2004))
    assert 615 not in got and 616 not in got and 2005 not in got


def test_keeps_the_systemd_policy_process():
    assert 1749 not in pids(find_strays(PS, self_pid=2004, keep_pids={1749}))


def test_healthy_graph_is_ready():
    assert problems(**GOOD) == []


def test_driver_not_discovered_yet_is_not_ready():
    # 2026-09-23 开机 1 分钟时的真实情况:只看得到 policy,订阅者 0
    bad = dict(GOOD, cmd_pubs=['steer_policy_node'], cmd_subs=[])
    assert any('racecar_driver' in p for p in problems(**bad))


def test_flags_each_failure():
    bad = dict(GOOD,
               units=dict(GOOD['units'], **{'racecar-chassis': 'activating'}),
               rates=dict(GOOD['rates'], **{'/scan': 0.0}),
               cmd_pubs=GOOD['cmd_pubs'] + ['ramp_curve'],
               scan_pubs=2, idle_cmds=40)
    text = '\n'.join(problems(**bad))
    for key in ('racecar-chassis', '/scan 0.0', 'ramp_curve', 'ROS_DOMAIN_ID', '40 条'):
        assert key in text, key


def test_esc_link_needs_the_current_device():
    fds = ['/dev/shm/fastrtps_x', 'anon_inode:[eventfd]', '/dev/ttyUSB2']
    assert esc_link_ok(fds, '/dev/ttyUSB2')


def test_esc_link_rejects_stale_fd_after_usb_reset():
    # 2026-09-23 实况:hub 复位后驱动攥着的是被删掉的旧节点
    assert not esc_link_ok(['/dev/ttyUSB2 (deleted)'], '/dev/ttyUSB2')


def test_esc_link_rejects_driver_that_never_opened():
    assert not esc_link_ok(['/dev/shm/fastrtps_x', 'pipe:[22845]'], '/dev/ttyUSB2')


def test_dead_esc_link_is_not_ready_even_if_sensors_are_fine():
    assert any('电调串口' in p for p in problems(**dict(GOOD, esc_ok=False)))


def test_probe_sends_true_neutral():
    # 探串口时喂的中位必须是真中位 1540,1550 带轻油门会让车爬
    assert PWM_MID == 1540


def test_own_neutral_probe_is_not_a_stranger():
    assert problems(**dict(GOOD, cmd_pubs=GOOD['cmd_pubs'] + ['carready'])) == []


def test_sleep_turned_back_on_is_not_ready():
    # 2026-09-23:桌面空闲挂起、唤醒后电调不响应,别的检查全过
    states = dict(GOOD['sleep_states'], **{'suspend.target': 'static'})
    found = problems(**dict(GOOD, sleep_states=states))
    assert len(found) == 1 and 'suspend.target' in found[0] and 'mask' in found[0]
