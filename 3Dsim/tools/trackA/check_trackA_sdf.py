#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_trackA_sdf.py —— 检查"trackA.world + 队友锥桶资产"与冻结 CSV 的一致性。

与 check_trackA.py 同一纪律:只读生成后的文件,不 import 生成器;独立解析
world XML 与 CSV 对账。坐标唯一事实来源仍是 docs/track/trackA_cones.csv。

锥桶资产契约(racecar_test_assets@7582feb,GPT 验收回执批准;2026-09-15 审定):
  包络=官方规则§3.4(高630mm/底280mm) | 原点=底座底面中心
  base box 0.28×0.28×0.05 | 锥台STL 96三角 z0→0.58 底r0.12/顶r0.04 装于z=0.05
  visual==collision 同一STL | 蓝/红几何字节相同仅material | static=true(测试专用)

检查项:
  1. world 结构:include 总数 = CSV 锥数;uri∈{model://cone_blue,cone_red};
     name=cone_<csv id>;pose (x,y) 与 CSV 逐锥 1e-6 对账(无序键匹配)
  2. 颜色对账:蓝/红 include 数 = CSV 蓝/红 数(37/26)
  3. 资产契约:model 文件在盘;static=true;base box 碰撞 0.28×0.28×0.05;
     mesh 碰撞存在且 visual==collision 同 uri;diffuse 蓝/红区分;
     蓝/红 STL 字节相同
  4. mesh 实测(二进制 STL 独立解析):三角数 96=文件长度核;z 0→0.58;
     max_r 0.12(在底座半宽 0.14 内)
  5. 官方包络断言:底座 0.28 + 总高 0.05+0.58=0.63(规则§3.4);
     锥台装配 z=底座厚;model.config 声明官方尺寸;asmp 残余披露

--negtest 定向断言(对 world 副本注入,须恰中对应检查项):
  N1 挪一锥 +0.1 m  → "world poses match CSV" 须 FAIL
  N2 蓝 include 改红 → "color counts match CSV" 须 FAIL
任何 FAIL 非零退出;负例未被抓住或抓错检查项也非零退出。
"""
import argparse
import copy
import csv
import math
import os
import re
import struct
import sys
import tempfile
try:
    from defusedxml.ElementTree import parse as safe_parse   # 装了就用防 XXE 版
except ImportError:
    from xml.etree.ElementTree import parse as safe_parse
import xml.etree.ElementTree as ET   # 仅 negtest 写副本用(受信本仓库文件)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..', '..', 'hwcart_sim',
                                    'src', 'racecar_sim'))
ASSETS = os.path.normpath(os.path.join(
    HERE, '..', '..', 'hwcart_sim', 'src', 'racecar_test_assets', 'models'))
WORLD = os.path.join(PKG, 'worlds', 'trackA.world')

# ---- 队友资产契约常量(7582feb 实测审定;检查器独立复核这些值) ----
EXPECT_DIFFUSE = {"blue": (0.05, 0.15, 0.85), "red": (0.85, 0.10, 0.10)}
TOL_POS = 1e-6
TOL_COL = 0.05
STL_TRIS = 96          # 锥台三角数
STL_ZMAX = 0.58        # 锥台高(总高0.63 − 底座0.05)
STL_RMAX = 0.12        # 锥台底半径(asmp,须 ≤ 底座半宽 0.14)
BASE_W = 0.28          # 官方:底座 280mm
BASE_H = 0.05          # asmp:底座厚
TOTAL_H = 0.63         # 官方:总高 630mm


class Rep:
    def __init__(self):
        self.fails = 0
        self.failed_names = set()

    def add(self, name, ok, detail=""):
        tag = "OK  " if ok else "FAIL"
        if not ok:
            self.fails += 1
            self.failed_names.add(name)
        print("  [%s] %-34s %s" % (tag, name, detail))
        return ok


def load_csv_cones(csv_path):
    with open(csv_path, encoding='utf-8', newline='') as f:
        return [{"id": r["id"].strip(), "color": r["color"].strip(),
                 "x": float(r["x"]), "y": float(r["y"])}
                for r in csv.DictReader(f)]


def parse_world(world_path):
    """[(name, uri_color, x, y)] ;场景基础设施(ground_plane/sun)不计入。
    raise ET.ParseError 如果 XML 坏。"""
    root = safe_parse(world_path).getroot()
    out = []
    for inc in root.iter("include"):
        uri = (inc.findtext("uri") or "").strip()
        if uri in ("model://ground_plane", "model://sun"):
            continue                     # 场景基础设施,不参与锥桶对账
        m = re.match(r"^model://cone_(blue|red)$", uri)
        color = m.group(1) if m else None
        pose = inc.findtext("pose", default="")
        parts = pose.split()
        x, y = (float(parts[0]), float(parts[1])) if len(parts) >= 2 else (None, None)
        out.append((inc.findtext("name", default=""), color, x, y))
    return out


def parse_stl(path):
    """二进制 STL → (三角数核验, z_max, r_max);raise ValueError 若结构坏。"""
    d = open(path, 'rb').read()
    if len(d) < 84:
        raise ValueError("stl too short")
    n = struct.unpack('<I', d[80:84])[0]
    if (len(d) - 84) != n * 50:
        raise ValueError("stl length %d != 84+50*%d" % (len(d), n))
    zmax = 0.0
    rmax = 0.0
    for i in range(n):
        off = 84 + i * 50
        for v in range(3):
            x, y, z = struct.unpack('<3f', d[off+12+v*12: off+24+v*12])
            zmax = max(zmax, z)
            rmax = max(rmax, math.hypot(x, y))
    return n, zmax, rmax


def run_checks(csv_path, world_path, label=""):
    rep = Rep()
    print("=" * 100)
    print("check_trackA_sdf %s" % ("[%s]" % label if label else ""))
    print("  csv  : %s" % csv_path)
    print("  world: %s" % world_path)
    print("  assets: %s" % ASSETS)

    cones = load_csv_cones(csv_path)
    nb = sum(1 for c in cones if c["color"] == "blue")
    nr = len(cones) - nb

    # 1 world 结构 + 位置逐锥对账
    print("-- 1) world include 结构与位置对账")
    incs = []
    try:
        incs = parse_world(world_path)
        rep.add("world xml well-formed", True)
    except Exception as e:
        rep.add("world xml well-formed", False, repr(e))
    bad_uri = [n for n, c, _, _ in incs if c is None]
    rep.add("include uri in cone_{blue,red}", not bad_uri, "bad: %s" % bad_uri[:4])
    rep.add("include count == csv count", len(incs) == len(cones),
            "world=%d csv=%d" % (len(incs), len(cones)))
    # 键控无序对账:(x,y)@1e-6 为位置键,name/color 逐项核
    key = lambda x, y: (round(x, 6), round(y, 6))
    csv_map = {key(c["x"], c["y"]): c for c in cones}
    wld_map = {key(x, y): (n, c) for n, c, x, y in incs if x is not None}
    pose_bad = []
    for k, c in csv_map.items():
        if k not in wld_map:
            pose_bad.append("csv %s (%s) missing in world" % (c["id"], k))
        else:
            n, col = wld_map[k]
            if n != "cone_%s" % c["id"]:
                pose_bad.append("name %s != cone_%s @%s" % (n, c["id"], k))
            if col != c["color"]:
                pose_bad.append("%s color %s != csv %s" % (n, col, c["color"]))
    extra = [k for k in wld_map if k not in csv_map]
    for k in extra:
        pose_bad.append("world cone @%s not in csv (%s)" % (k, wld_map[k][0]))
    rep.add("world poses match CSV", not pose_bad, "; ".join(pose_bad[:4]))

    # 2 颜色计数
    print("-- 2) 颜色对账")
    wb = sum(1 for _, c, _, _ in incs if c == "blue")
    wr = sum(1 for _, c, _, _ in incs if c == "red")
    rep.add("color counts match CSV",
            wb == nb and wr == nr,
            "world %d blue/%d red vs csv %d/%d" % (wb, wr, nb, nr))

    # 3 资产契约(队友 cone_{blue,red} model)
    print("-- 3) 资产契约(文件/static/base碰撞/mesh碰撞=visual/diffuse/同构)")
    stl_bytes = {}
    for color in ("blue", "red"):
        d = os.path.join(ASSETS, 'cone_%s' % color)
        stl_p = os.path.join(d, 'meshes', 'cone_frustum.stl')
        ok_files = (os.path.isfile(os.path.join(d, 'model.config')) and
                    os.path.isfile(os.path.join(d, 'model.sdf')) and
                    os.path.isfile(stl_p))
        rep.add("%s model files exist" % color, ok_files, d)
        if not ok_files:
            continue
        stl_bytes[color] = open(stl_p, 'rb').read()
        try:
            root = safe_parse(os.path.join(d, 'model.sdf')).getroot()
        except ET.ParseError as e:
            rep.add("%s model.sdf well-formed" % color, False, str(e))
            continue
        # static=true(契约:测试专用,不可验证推移)
        st = (root.findtext("model/static") or "").strip()
        rep.add("%s static=true (test-only)" % color, st == "true",
                "static=%r" % st)
        # base box 碰撞 0.28×0.28×0.05
        box_bad = True
        for c in root.iter("collision"):
            sz = (c.findtext("geometry/box/size") or "").split()
            if len(sz) == 3 and (abs(float(sz[0]) - BASE_W) < 1e-9 and
                                 abs(float(sz[1]) - BASE_W) < 1e-9 and
                                 abs(float(sz[2]) - BASE_H) < 1e-9):
                box_bad = False
        rep.add("%s base box collision %.2fx%.2fx%.2f" % (color, BASE_W, BASE_W, BASE_H),
                not box_bad, "" if not box_bad else "not found")
        # mesh 碰撞存在且 visual==collision 同 uri(同 mesh 契约)
        col_uris = set()
        for c in root.iter("collision"):
            u = c.findtext("geometry/mesh/uri")
            if u:
                col_uris.add(u.strip())
        vis_uris = set()
        for v in root.iter("visual"):
            u = v.findtext("geometry/mesh/uri")
            if u:
                vis_uris.add(u.strip())
        rep.add("%s mesh collision present" % color, len(col_uris) > 0,
                sorted(col_uris))
        rep.add("%s visual==collision mesh" % color,
                bool(col_uris) and col_uris == vis_uris,
                "col=%s vis=%s" % (sorted(col_uris), sorted(vis_uris)))
        # diffuse 区分
        diffuse_bad = []
        for mat in root.iter("material"):
            df = mat.findtext("diffuse", default="")
            if not df.strip():
                continue
            try:
                rgba = [float(v) for v in df.split()]
            except ValueError:
                diffuse_bad.append("unparseable '%s'" % df)
                continue
            exp = EXPECT_DIFFUSE[color]
            if any(abs(rgba[i]-exp[i]) > TOL_COL for i in range(3)):
                diffuse_bad.append("diffuse %s vs expect %s" % (rgba[:3], exp))
        rep.add("%s diffuse matches color" % color, not diffuse_bad,
                "; ".join(diffuse_bad[:3]))
    same = ("blue" in stl_bytes and "red" in stl_bytes and
            stl_bytes["blue"] == stl_bytes["red"])
    rep.add("blue/red STL byte-identical", same,
            "" if same else "geometry diverged between colors!")

    # 4 mesh 实测(二进制 STL 独立解析)
    print("-- 4) 锥台 STL 实测(三角数/z包络/底半径)")
    try:
        n, zmax, rmax = parse_stl(os.path.join(ASSETS, 'cone_blue',
                                               'meshes', 'cone_frustum.stl'))
        rep.add("stl triangles == %d" % STL_TRIS, n == STL_TRIS,
                "n=%d" % n)
        rep.add("stl zmax == %.2f" % STL_ZMAX,
                abs(zmax - STL_ZMAX) < 1e-4, "zmax=%.6f" % zmax)
        rep.add("stl rmax == %.2f (<=base/2)" % STL_RMAX,
                abs(rmax - STL_RMAX) < 1e-4 and rmax <= BASE_W/2 + 1e-9,
                "rmax=%.6f" % rmax)
    except Exception as e:
        rep.add("stl parse", False, repr(e))

    # 5 官方包络断言 + 装配一致性 + 披露
    print("-- 5) 官方包络(base 0.28+0.05+0.58=0.63;装配z=底座厚;config披露)")
    asm_bad = []
    for color in ("blue", "red"):
        try:
            root = safe_parse(os.path.join(ASSETS, 'cone_%s' % color,
                                           'model.sdf')).getroot()
        except Exception as e:
            asm_bad.append("%s parse %r" % (color, e))
            continue
        zs = []
        for tag in ("collision", "visual"):
            for e2 in root.iter(tag):
                u = e2.findtext("geometry/mesh/uri")
                if u:                       # 锥台件:装配 z 应=底座厚
                    p = (e2.findtext("pose") or "0 0 0 0 0 0").split()
                    zs.append(float(p[2]))
        if any(abs(z - BASE_H) > 1e-9 for z in zs) or not zs:
            asm_bad.append("%s frustum mount z=%s (!= base_h %.2f)"
                           % (color, zs, BASE_H))
    rep.add("frustum mounted at base top", not asm_bad,
            "; ".join(asm_bad[:3]))
    cfg = open(os.path.join(ASSETS, 'cone_blue', 'model.config'),
               encoding='utf-8').read()
    sdf_txt = open(os.path.join(ASSETS, 'cone_blue', 'model.sdf'),
                   encoding='utf-8').read()
    rep.add("config declares 0.63/0.28 rule dims",
            ("0.63" in cfg and "0.28" in cfg), "")
    rep.add("asmp residue disclosed", "assumption" in cfg or "assumption" in sdf_txt,
            "(base thickness/frustum diameters)")

    print("--\nTOTAL FAIL(s): %d" % rep.fails)
    return rep.fails, rep.failed_names


def negtest(csv_path, world_path):
    misses = 0
    tree = ET.parse(world_path)
    tmpdir = tempfile.mkdtemp(prefix="trackA_sdf_neg_")
    # N1: 挪第一个锥桶 +0.1 m(跳过 ground_plane/sun 等基础设施 include)
    p1 = os.path.join(tmpdir, "neg1.world")
    t1 = copy.deepcopy(tree)
    inc1 = next((inc for inc in t1.getroot().iter("include")
                 if "model://cone_" in (inc.findtext("uri") or "")), None)
    if inc1 is None:
        print("  NEG-1 MISSED!! (world 里没有 cone_ include,无从注入)\n")
        return misses + 1
    pose = inc1.findtext("pose").split()
    pose[0] = "%.6f" % (float(pose[0]) + 0.1)
    inc1.find("pose").text = " ".join(pose)
    t1.write(p1, encoding="utf-8", xml_declaration=True)
    f1, n1 = run_checks(csv_path, p1, label="NEG-1 cone shifted +0.1m")
    caught1 = f1 > 0 and "world poses match CSV" in n1
    print("  NEG-1 %s (fails=%d, 'world poses match CSV' failed: %s)\n" %
          ("CAUGHT" if caught1 else "MISSED!!", f1,
           "yes" if "world poses match CSV" in n1 else "no"))
    misses += (1 if not caught1 else 0)

    # N2: 把一个蓝锥 uri 改成红
    p2 = os.path.join(tmpdir, "neg2.world")
    t2 = copy.deepcopy(tree)
    for inc in t2.getroot().iter("include"):
        if (inc.findtext("uri") or "").strip() == "model://cone_blue":
            inc.find("uri").text = "model://cone_red"
            break
    t2.write(p2, encoding="utf-8", xml_declaration=True)
    f2, n2 = run_checks(csv_path, p2, label="NEG-2 blue include recolored red")
    caught2 = f2 > 0 and "color counts match CSV" in n2
    print("  NEG-2 %s (fails=%d, 'color counts match CSV' failed: %s)\n" %
          ("CAUGHT" if caught2 else "MISSED!!", f2,
           "yes" if "color counts match CSV" in n2 else "no"))
    misses += (1 if not caught2 else 0)
    return misses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.normpath(os.path.join(
        HERE, '..', '..', 'docs', 'track', 'trackA_cones.csv')))
    ap.add_argument("--world", default=WORLD)
    ap.add_argument("--negtest", action="store_true")
    a = ap.parse_args()
    fails, _ = run_checks(a.csv, a.world)
    extra = negtest(a.csv, a.world) if a.negtest else 0
    if fails or extra:
        print("EXIT: nonzero (FAIL present)" if fails else
              "EXIT: nonzero (negative case MISSED)")
        sys.exit(1)
    print("EXIT: 0")
    sys.exit(0)


if __name__ == "__main__":
    main()
