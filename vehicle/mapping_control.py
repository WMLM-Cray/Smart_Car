"""Manage only the exact GMapping executable, never its chassis-launching parent."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import zipfile

EXECUTABLE = Path.home() / 'racecar/install/slam_gmapping/lib/slam_gmapping/slam_gmapping'


def mapper_pids():
    result = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            if (item / 'exe').resolve(strict=True) == EXECUTABLE.resolve(strict=True):
                result.append(int(item.name))
        except (OSError, RuntimeError):
            pass
    return result


class MappingControl:
    def __init__(self, publisher_count):
        self.lock = threading.Lock()
        self.publisher_count = publisher_count
        self.busy = False
        self.message = ''
        self.saved = ''
        self.child = None

    def status(self):
        with self.lock:
            if self.child:
                self.child.poll()  # Reap an exited owned child.
            return dict(running=bool(mapper_pids()), busy=self.busy,
                        message=self.message, saved=self.saved)

    def request(self, action):
        if action not in ('start', 'stop'):
            raise ValueError('Unknown mapping action')
        with self.lock:
            if self.busy:
                return False
            self.busy = True
            self.message = '正在启动…' if action == 'start' else '正在保存，请保持车停稳…'
        threading.Thread(target=self._work, args=(action,), daemon=True).start()
        return True

    def _work(self, action):
        try:
            if action == 'start':
                self._start()
            else:
                self._stop()
        except Exception as exc:
            with self.lock:
                self.message = '操作失败：' + str(exc)[-500:]
        finally:
            with self.lock:
                self.busy = False

    def _start(self):
        if mapper_pids():
            with self.lock:
                self.message = '建图已运行，未重复启动'
            return
        if self.publisher_count():
            raise RuntimeError('/map 已有其他发布者，拒绝重复启动')
        with open(Path(__file__).with_name('mapping.log'), 'ab') as log:
            self.child = subprocess.Popen([str(EXECUTABLE), '--ros-args',
                '-p', 'use_sim_time:=false', '-p', 'base_frame:=base_footprint',
                '-p', 'odom_frame:=odom_combined', '-p', 'map_frame:=map',
                '-p', 'laser_frame:=laser_link'], stdout=log, stderr=log,
                stdin=subprocess.DEVNULL, start_new_session=True,
                # 降优先级:GMapping 一个进程就吃满一个核(2026-09-24 负载 19),和策略/网页/
                # 看门狗抢 CPU 时网页按钮点了没反应。建图慢一点没关系,控制链不能卡。
                preexec_fn=lambda: os.nice(15))
        time.sleep(1)
        if self.child.poll() is not None:
            raise RuntimeError('建图启动失败，请检查 mapping.log')
        with self.lock:
            self.message = '新地图已启动，等待扫描生成 /map'

    def _stop(self):
        pids = mapper_pids()
        if len(pids) != 1:
            raise RuntimeError('需要且只能有一个 GMapping 进程；未停止任何进程')
        # Bind signal delivery to this process identity, not a reusable numeric PID.
        fd = os.pidfd_open(pids[0])
        try:
            if mapper_pids() != pids:
                raise RuntimeError('建图进程已变化，请重试')
            if self.publisher_count() != 1:
                raise RuntimeError('/map 发布者数量异常，拒绝保存后停机')
            result = subprocess.run([sys.executable, '-m', 'vehicle.export_map'],
                                    capture_output=True, text=True, timeout=30, check=True)
            payload = json.loads(result.stdout.strip().splitlines()[-1])
            path = Path(payload['file']).resolve()
            if path.parent != (Path.home() / 'maps').resolve():
                raise RuntimeError('导出文件路径异常')
            with zipfile.ZipFile(path) as archive:
                if not {'map.pgm', 'map.yaml'} <= set(archive.namelist()) or archive.testzip():
                    raise RuntimeError('地图文件校验失败，保持建图运行')
            with self.lock:
                self.saved = str(path)
            signal.pidfd_send_signal(fd, signal.SIGINT)
            for _ in range(50):
                if self.child:
                    self.child.poll()
                if not mapper_pids():
                    with self.lock:
                        self.message = '地图已保存，建图已停止'
                    return
                time.sleep(.1)
            raise RuntimeError('地图已保存，但建图进程未退出；没有强制终止')
        finally:
            os.close(fd)
