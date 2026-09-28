#!/usr/bin/env python3
"""把官方赛道 A-D 转成 Gazebo 用的 layout.yaml + track_sim.world。

**锥桶和起点直接从本仓库的 `evaluation.road_track.official(name, fill_v=True)` 取**,
不自己解析 maps/ 下的 CSV/JSON。理由:fastsim 验收用的就是这一个函数,从它取
才能保证 Gazebo 里的赛道与 fastsim 验收时**逐点一致** —— 差异只可能来自物理,
不会来自赛道。

坐标约定(2026-09-28 定):**保留 maps/ 原坐标**,不旋转不平移 —— Gazebo、FastSim、
maps 里同一个点坐标相同,失败位置能直接对照(比如 A 左上方角两边都是 (0.5, 4.0))。
  * official() 返回的中心线 cen[0] 是 START 框中心(底边直道),heading[0] 是该点
    切向;写进 layout.yaml 的 start_pose(xy + yaw_deg)。
  * run_road_policy_once.sh / run_s3_policy_once.sh 从 layout 的 start_pose 取 spawn,
    track_eval.py 也用它判过线 —— 两处用同一个起点,所以不需要归零。
  * spawn 和赛道对不上的后果(2026-09-25 实测):车生成在赛道中段随机位置,往往紧贴
    锥桶,一启动就被 ScanGuard 判截停 —— 看起来像模型崩了,其实是坐标不匹配。

黄 V:fastsim 验收用的是 `fill_v=True`(沿边界按 0.5m 补桶),这里必须同样传
True,否则 Gazebo 里的赛道和验收的那条不是同一条。

用法:
    python3 simulation/gazebo/official_to_gazebo.py --out runs/gz_official
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np                      # noqa: E402
import yaml                             # noqa: E402

from evaluation.road_track import official   # noqa: E402

HEAD = """<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
  <world name="prototype">
    <gravity>0 0 -9.81</gravity>
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <light name="sun" type="directional">
      <direction>-0.5 0.2 -1</direction>
      <diffuse>1 1 1 1</diffuse>
    </light>
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="body">
        <visual name="visual">
          <geometry>
            <box>
              <size>200 200 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>.35 .35 .35 1</ambient>
            <diffuse>.35 .35 .35 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>200 200 0.1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
"""

CONE = """    <model name="cone_proxy_{cid}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.315 0 0 0</pose>
      <link name="body">
        <visual name="visual">
          <geometry>
            <box>
              <size>.28 .28 .63</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>.28 .28 .63</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
"""

TAIL = """    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
  </world>
</sdf>
"""


def convert(name: str):
    """maps/ 原坐标:锥桶、中心线原样,起点 = START 框中心和该点切向。"""
    road, lap_len = official(name, fill_v=True)
    cen = np.asarray(road.centre, float)
    return dict(name=name, lap_len=lap_len, cones=np.asarray(road.cones, float), centre=cen,
                half=np.asarray(road.half, float),
                start=(float(cen[0, 0]), float(cen[0, 1]), float(road.heading[0])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--tracks', nargs='+', default=['A', 'B', 'C', 'D'])
    a = ap.parse_args()

    root = Path(a.out)
    manifest = []
    for name in a.tracks:
        d = root / f'track_{name}'
        d.mkdir(parents=True, exist_ok=True)
        c = convert(name)
        lay_cones = [dict(id=f'c{i}', x=round(float(x), 4), y=round(float(y), 4))
                     for i, (x, y) in enumerate(c['cones'])]
        sx, sy, syaw = c['start']
        lay = dict(track=f'official {name} (fill_v=True)',
                   start_pose=dict(xy=[round(sx, 4), round(sy, 4)],
                                   yaw_deg=round(math.degrees(syaw), 3)),
                   cones=lay_cones)
        (d / 'layout.yaml').write_text(
            yaml.safe_dump(lay, allow_unicode=True, sort_keys=False), encoding='utf-8')
        parts = [HEAD]
        for cc in lay_cones:
            parts.append(CONE.format(cid=cc['id'], x=cc['x'], y=cc['y']))
        parts.append(TAIL)
        (d / 'track_sim.world').write_text(''.join(parts), encoding='utf-8')
        # 中心线/半宽另存一份,画轨迹和诊断用
        np.save(d / 'centerline.npy', c['centre'])
        np.save(d / 'half.npy', c['half'])
        info = dict(track=name, cones=len(lay_cones), lap_len=c['lap_len'],
                    half_min=float(c['half'].min()), half_max=float(c['half'].max()),
                    dir=str(d))
        manifest.append(info)
        print(f'  track_{name}: {info["cones"]} 桶  一圈 {c["lap_len"]:.2f}m  '
              f'半宽 {info["half_min"]:.3f}~{info["half_max"]:.3f}m -> {d}', flush=True)
    (root / 'manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'-> {root}/manifest.json')


if __name__ == '__main__':
    main()
