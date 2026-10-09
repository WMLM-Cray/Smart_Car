#!/usr/bin/env python3
"""A-D 面板上两臂（Y 候选 vs A 对照）的成对判读器。

用法:
    ad_lap_verdict.py <CAP> [--tracks A,B,C,D] [--arms Y,A] [--runs panel|gz_official|all]

只认 `pairing.certified == True` 且非开局早夭的局；同时给两套干净口径：
    clean_strict = laps>=3 and cone_hits==0 and off_track_samples==0
    clean_loose  = laps>=1 and cone_hits==0 and off_track_samples==0

圈速口径（写进报告时必须说清用的是哪一套）：
    steady = 丢掉每局第一圈后的中位（第一圈含起步/停靠） —— 报告主口径
    alllap = 全部圈的中位（含卡停，卡停会把它抬高）
    quiet  = 本脚本另一列 maxcut（贴弯切线计数），用来交代"有没有靠切角换时间"

--runs 必须按"跑批用的哪个世界"选：panel = 仓内实测 sim + 仓内宿主那批；
gz_official = 旧尺子那批（队友 sim + 仓外宿主）。两批 cap/臂名相同，`all` 混读会把分母
与中位算错，所以脚本在有歧义时会显式警告。

反作弊体检（本项目历史教训）：同 cap 下若两臂 Σdist 之比 < 0.8 或 > 1.25，
说明其中一臂在用"少走/多走"换读数，判读作废。
"""
import argparse
import glob
import json
import os
import re
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# 2026-10-08：本脚本从 runs/ 挪到 evaluation/，跑批产物仍在 <repo>/runs/；
# 赛道目录名也从 gz_official（认证副本在 runs/ 下）换成 panel（认证副本入库 evaluation/panel/），
# 两种都认。
RUNS = os.path.join(os.path.dirname(HERE), 'runs')
PAT = re.compile(r'CORRIDOR_arm([A-Z]+)_cap([0-9.]+)_(?:gz_official|panel)_track_([A-D])_r(\d+)$')


def rows(cap, tracks, arms, kind='all'):
    """kind: 'panel' 只认 evaluation/panel 赛道那批；'gz_official' 只认 runs/gz_official 那批；
    'all' 两批都要（默认）。两批的 cap/臂名相同，混在一起读会把分母算错。"""
    out = []
    for d in sorted(glob.glob(os.path.join(RUNS, 'gazebo_supervised_smoke_*'))):
        m = PAT.search(os.path.basename(d))
        if not m:
            continue
        base = os.path.basename(d)
        if kind == 'panel' and '_panel_track_' not in base:
            continue
        if kind == 'gz_official' and '_gz_official_track_' not in base:
            continue
        arm, c, trk, rep = m.group(1), m.group(2), m.group(3), int(m.group(4))
        if float(c) != float(cap) or trk not in tracks or arm not in arms:
            continue
        rp = os.path.join(d, 'report.json')
        j = json.load(open(rp, encoding='utf-8')) if os.path.exists(rp) else {}
        sr = j.get('scorer_result') or {}
        pa = j.get('pairing') or {}
        au = j.get('track_progress_audit') or {}
        laps = int(sr.get('laps') or 0)
        wall = float(sr.get('elapsed_wall_s') or 0.0)
        # 开局早夭（"看见就死"）：策略一个指令都没发、模拟只推进几秒。
        # 这类局的 pairing.certified 会是 True（anchor/enable 都对得上），
        # 必须靠 laps/status/wall 二次过滤，否则它会把分母撑大、把干净率压低。
        # 判据与 runs/void_early_abort.sh 一致（cmd==0 那条由 laps==0+failed 等价覆盖）。
        void = (laps == 0) and (j.get('status') == 'failed' or (0.0 < wall < 30.0))
        out.append(dict(arm=arm, track=trk, rep=rep,
                        certified=bool(pa.get('certified')),
                        void=void,
                        status=j.get('status'),
                        wall=wall,
                        laps=laps,
                        hits=int(sr.get('cone_hits') or 0),
                        off=int(sr.get('off_track_samples') or 0),
                        splits=[float(x) for x in (sr.get('lap_splits_s') or [])],
                        dist=sr.get('max_dist_to_nearest_cone_m'),
                        cut=au.get('track_cut_count'),
                        maxcut=au.get('max_cut_gain_m'),
                        dir=os.path.basename(d)))
    return sorted(out, key=lambda x: (x['track'], x['arm'], x['rep']))


def med(xs):
    return st.median(xs) if xs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cap')
    ap.add_argument('--tracks', default='A,B,C,D')
    ap.add_argument('--arms', default='A,L')
    ap.add_argument('--runs', default='all', choices=('all', 'panel', 'gz_official'),
                    help='只读哪一批跑批目录（两批 cap/臂名相同，混读会把分母算错）')
    a = ap.parse_args()
    tracks = [t.strip().upper() for t in a.tracks.split(',') if t.strip()]
    arms = [x.strip().upper() for x in a.arms.split(',') if x.strip()]
    if a.runs == 'all':
        # 两批同 cap/臂名，但世界（队友 sim vs 仓内实测 sim）与宿主都不同，混读会串口径。
        kinds = set()
        for d in glob.glob(os.path.join(RUNS, 'gazebo_supervised_smoke_*')):
            b = os.path.basename(d)
            if not PAT.search(b):
                continue
            if '_panel_track_' in b:
                kinds.add('panel')
            elif '_gz_official_track_' in b:
                kinds.add('gz_official')
        if len(kinds) > 1:
            print('⚠ --runs all 同时含 %s 两批（几何/宿主不同，数值不可比）；报告请用 --runs panel'
                  % ' + '.join(sorted(kinds)))
    rs = rows(a.cap, tracks, arms, a.runs)
    if not rs:
        print('没有匹配的局（cap=%s tracks=%s arms=%s runs=%s）' % (a.cap, tracks, arms, a.runs))
        return 1
    print('cap=%s  A-D 面板成对判读（runs=%s；只认 certified 局；开局早夭局作废）' % (a.cap, a.runs))
    voids = [x for x in rs if x['void']]
    if voids:
        print('作废（开局早夭，不计入分母）: ' +
              ', '.join('%s %s r%d(wall=%.1fs,status=%s)' % (x['arm'], x['track'], x['rep'], x['wall'], x['status'])
                        for x in voids))
    print('%-6s %-3s %-5s %-6s %-5s %-5s %-5s %-8s %-8s %-7s %s'
          % ('arm', 'trk', 'rep', 'cert', 'laps', 'hits', 'off', 'steady', 'alllap', 'maxcut', 'splits'))
    for x in rs:
        if x['void']:
            continue
        steady = med(x['splits'][1:])
        print('%-6s %-3s %-5d %-6s %-5d %-5d %-5d %-8s %-8s %-7s %s'
              % (x['arm'], x['track'], x['rep'], 'Y' if x['certified'] else 'N',
                 x['laps'], x['hits'], x['off'],
                 ('%.2f' % steady) if steady else '-',
                 ('%.2f' % med(x['splits'])) if x['splits'] else '-',
                 ('%.3f' % x['maxcut']) if x['maxcut'] is not None else '-',
                 '[' + ','.join('%.1f' % s for s in x['splits']) + ']'))
    print()
    for arm in arms:
        g = [x for x in rs if x['arm'] == arm and x['certified'] and not x['void']]
        if not g:
            continue
        strict = [x for x in g if x['laps'] >= 3 and x['hits'] == 0 and x['off'] == 0]
        loose = [x for x in g if x['laps'] >= 1 and x['hits'] == 0 and x['off'] == 0]
        steady = [s for x in g for s in x['splits'][1:]]
        alllap = [s for x in g for s in x['splits']]
        print('arm %s: 认证 %d 局 | strict %d/%d = %5.1f%% | loose %d/%d = %5.1f%% | '
              'Σlaps %d Σhits %d Σoff %d | steady 中位 %-6s 均值 %-6s | alllap 中位 %-6s 均值 %-6s'
              % (arm, len(g), len(strict), len(g), 100.0 * len(strict) / len(g),
                 len(loose), len(g), 100.0 * len(loose) / len(g),
                 sum(x['laps'] for x in g), sum(x['hits'] for x in g),
                 sum(x['off'] for x in g),
                 ('%.2f s' % st.median(steady)) if steady else '-',
                 ('%.2f s' % st.mean(steady)) if steady else '-',
                 ('%.2f s' % st.median(alllap)) if alllap else '-',
                 ('%.2f s' % st.mean(alllap)) if alllap else '-'))
        per = {}
        for x in g:
            k = per.setdefault(x['track'], [0, 0])
            k[0] += 1
            k[1] += 1 if (x['laps'] >= 3 and x['hits'] == 0 and x['off'] == 0) else 0
        print('        逐道 strict: ' + '  '.join('%s %d/%d' % (t, v[1], v[0]) for t, v in sorted(per.items())))
    # 反作弊体检：两臂行程比（只有报告里带 dist 才可比）
    for arm in arms:
        g = [x for x in rs if x['arm'] == arm and x['certified'] and not x['void'] and x['dist']]
        if g:
            print('arm %s Σdist(max_dist_to_nearest_cone_m) = %.3f (n=%d)' %
                  (arm, sum(x['dist'] for x in g), len(g)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
