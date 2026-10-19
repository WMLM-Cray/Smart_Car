"""遥测发送不许被一个收不动的客户端拖住。python3 -m pytest vehicle/test_ws_send.py

server.py 顶层 import rclpy,这里用 AST 只摘 ws_send / log_event 出来跑。
MSG_DONTWAIT 是 Linux 的,非 Linux 上跳过(车上跑):Windows 没有这个常量;macOS 有,但 socketpair
(AF_UNIX)缓冲满了照样阻塞,测试会卡死。
"""
import ast
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='需要 Linux')


@pytest.fixture
def ns(tmp_path):
    tree = ast.parse(Path(__file__).with_name('server.py').read_text(encoding='utf-8'))
    keep = [n for n in tree.body if getattr(n, 'name', '') in ('ws_send', 'log_event')]
    out = {'socket': socket, 'struct': struct, 'time': time,
           'EVENTS': str(tmp_path / 'events.log')}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'server_ws', 'exec'), out)
    return out


def test_stuck_client_is_dropped_not_waited_on(ns):
    a, b = socket.socketpair()          # b 永远不读 = 半死不活的页面
    conn = (a, threading.Lock())
    t0 = time.time()
    for _ in range(20000):
        try:
            ns['ws_send'](conn, 'x' * 300, wait=False)
        except OSError:
            break                       # 已经 shutdown,再发就报错 —— 遥测循环会吞掉
    assert time.time() - t0 < 2.0, '发给收不动的客户端时阻塞了'
    with pytest.raises(OSError):
        a.send(b'x')                    # 被断开了
    assert '断开' in Path(ns['EVENTS']).read_text()


def test_healthy_client_gets_whole_frames(ns):
    a, b = socket.socketpair()
    ns['ws_send']((a, threading.Lock()), '{"spd":0}', wait=False)
    assert b.recv(100) == bytes([0x81, 9]) + b'{"spd":0}'
