#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_trackA_sdf.py —— 检查"生成的 SDF 资产"与冻结 CSV 的一致性。

与 check_trackA.py 同一纪律:只读生成后的文件,不 import 生成器;独立解析
world XML 与 CSV 对账。SDF 侧唯一事实来源仍是 docs/track/trackA_cones.csv。

检查项:
  1. world 结构:include 总数 = CSV 锥数;uri∈{trackA_cone_blue,trackA_cone_red};
     name=cone_<csv id>;pose (x,y) 与 CSV 逐锥 1e-6 对账(无序键匹配)
  2. 颜色对账:蓝/红 include 数 = CSV 蓝/红 数(37/26)
  3. model 资产:两色 model.sdf 存在且良构;diffuse 区分(blue≈(0.15,0.40,0.90)
     / red≈(0.90,0.15,0.15),容差 0.05);collision=cylinder;visual mesh uri
     指向本 model meshes/ 且文件在盘
  4. mesh:dae XML 良构;三角形数 = 棱数×2(侧面+底面);法线数组长度=位置数组;
     顶点实测包络 max_r=0.14 / max_z=0.63(官方 280/630mm,独立于生成器参数)
  5. 官方尺寸断言:model.sdf collision 圆柱 r=0.14 h=0.63、底座 box 0.28×0.28;
     static=false(动态可撞);残余 asmp(底座厚/质量)须在生成器与 model.config 披露

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
WORLD = os.path.join(PKG, 'worlds', 'trackA.world')
N_SIDES = 16
EXPECT_DIFFUSE = {"blue": (0.15, 0.40, 0.90), "red": (0.90, 0.15, 0.15)}
TOL_POS = 1e-6
TOL_COL = 0.05
# 官方锥桶尺寸(规则§3.4:高630mm 底径280mm;碰撞圆柱=底径/2×锥高)
OFF_R, OFF_H, OFF_W = 0.14, 0.63, 0.28


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
        m = re.match(r"^model://trackA_cone_(blue|red)$", uri)
        color = m.group(1) if m else None
        pose = inc.findtext("pose", default="")
        parts = pose.split()
        x, y = (float(parts[0]), float(parts[1])) if len(parts) >= 2 else (None, None)
        out.append((inc.findtext("name", default=""), color, x, y))
    return out


def run_checks(csv_path, world_path, label=""):
    rep = Rep()
    print("=" * 100)
    print("check_trackA_sdf %s" % ("[%s]" % label if label else ""))
    print("  csv  : %s" % csv_path)
    print("  world: %s" % world_path)

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
    rep.add("include uri in {blue,red}", not bad_uri, "bad: %s" % bad_uri[:4])
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
    print("-- 2) 颜色计数对账")
    wb = sum(1 for _, c, _, _ in incs if c == "blue")
    wr = sum(1 for _, c, _, _ in incs if c == "red")
    rep.add("color counts match CSV",
            wb == nb and wr == nr,
            "world %d blue/%d red vs csv %d/%d" % (wb, wr, nb, nr))

    # 3 model 资产
    print("-- 3) model 资产(两色 model.sdf/颜色区分/collision/mesh 在盘)")
    for color in ("blue", "red"):
        d = os.path.join(PKG, 'models', 'trackA_cone_%s' % color)
        sdf_p = os.path.join(d, 'model.sdf')
        ok_files = (os.path.isfile(os.path.join(d, 'model.config')) and
                    os.path.isfile(sdf_p) and
                    os.path.isfile(os.path.join(d, 'meshes', 'traffic_cone.dae')))
        rep.add("%s model files exist" % color, ok_files, d)
        if not ok_files:
            continue
        try:
            root = safe_parse(sdf_p).getroot()
        except ET.ParseError as e:
            rep.add("%s model.sdf well-formed" % color, False, str(e))
            continue
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
        has_col = any(c.find("geometry/cylinder") is not None
                      for c in root.iter("collision"))
        rep.add("%s collision cylinder present" % color, has_col)
        mesh_ok, mesh_detail = True, ""
        for mu in root.iter("mesh"):
            uri = (mu.findtext("uri") or "").strip()
            mfile = uri.replace("model://trackA_cone_%s" % color, d)
            if not os.path.isfile(mfile):
                mesh_ok, mesh_detail = False, "missing %s" % uri
        rep.add("%s mesh file on disk" % color, mesh_ok, mesh_detail)

    # 4 mesh 几何数 + 实测包络(独立于生成器参数,直接从顶点算)
    print("-- 4) 锥 mesh(棱数×2 三角/法线配对/包络=官方 630/280)")
    dae_p = os.path.join(PKG, 'models', 'trackA_cone_blue',
                         'meshes', 'traffic_cone.dae')
    NS = "{http://www.collada.org/2005/11/COLLADASchema}"   # dae 带 ns
    try:
        droot = safe_parse(dae_p).getroot()
        fa = {e.get("id"): e for e in droot.iter("%sfloat_array" % NS)}
        pos_cnt = int(fa["positions-array"].get("count"))
        nrm_cnt = int(fa["normals-array"].get("count"))
        tris = sum(int(t.get("count")) for t in droot.iter("%striangles" % NS))
        rep.add("dae verts/normals paired", pos_cnt == nrm_cnt,
                "%d/%d" % (pos_cnt, nrm_cnt))
        rep.add("dae triangles = sides*2", tris == N_SIDES*2,
                "%d (expect %d)" % (tris, N_SIDES*2))
        vals = [float(v) for v in fa["positions-array"].text.split()]
        pts = [vals[i:i+3] for i in range(0, len(vals), 3)]
        max_r = max(math.sqrt(p[0]*p[0] + p[1]*p[1]) for p in pts)
        max_z = max(p[2] for p in pts)
        rep.add("dae extent = 630/280 official",
                abs(max_r - OFF_R) < 1e-6 and abs(max_z - OFF_H) < 1e-6,
                "max_r=%.6f (expect %.2f) max_z=%.6f (expect %.2f)"
                % (max_r, OFF_R, max_z, OFF_H))
    except Exception as e:
        rep.add("dae parse", False, repr(e))

    # 5 官方尺寸断言(collision/base/static)+ 残余 asmp 披露
    print("-- 5) 官方尺寸断言(cyl r=0.14 h=0.63;base 0.28;static=false;asmp 披露)")
    off_bad, dyn_bad = [], []
    for color in ("blue", "red"):
        sdf_p = os.path.join(PKG, 'models', 'trackA_cone_%s' % color,
                             'model.sdf')
        try:
            root = safe_parse(sdf_p).getroot()
        except Exception as e:
            off_bad.append("%s parse %r" % (color, e))
            continue
        st = (root.findtext("model/static") or "").strip()
        if st != "false":
            dyn_bad.append("%s static=%r" % (color, st))
        for c in root.iter("collision"):
            cy = c.find("geometry/cylinder")
            if cy is None:
                continue
            rr = float(cy.findtext("radius"))
            ll = float(cy.findtext("length"))
            if abs(rr - OFF_R) > 1e-9 or abs(ll - OFF_H) > 1e-9:
                off_bad.append("%s cyl r=%.4f l=%.4f" % (color, rr, ll))
        for v in root.iter("visual"):
            if v.get("name") != "base":
                continue
            sz = (v.findtext("geometry/box/size") or "").split()
            if len(sz) >= 2 and (abs(float(sz[0]) - OFF_W) > 1e-9 or
                                 abs(float(sz[1]) - OFF_W) > 1e-9):
                off_bad.append("%s base box %s" % (color, " ".join(sz)))
    rep.add("collision/base = official 630/280", not off_bad,
            "; ".join(off_bad[:4]))
    rep.add("cones dynamic (static=false)", not dyn_bad,
            "; ".join(dyn_bad[:4]))
    gen_p = os.path.join(HERE, 'trackA_sdf.py')
    disc = True
    detail = ""
    try:
        with open(gen_p, encoding='utf-8') as f:
            src = f.read()
        if "asmp" not in src or "simulation assumptions" not in \
                open(os.path.join(PKG, 'models', 'trackA_cone_blue',
                                  'model.config'), encoding='utf-8').read():
            disc, detail = False, "asmp residue disclosure missing"
    except OSError as e:
        disc, detail = False, str(e)
    rep.add("asmp residue disclosed", disc, detail)

    print("--\nTOTAL FAIL(s): %d" % rep.fails)
    return rep.fails, rep.failed_names


def negtest(csv_path, world_path):
    misses = 0
    tree = ET.parse(world_path)
    tmpdir = tempfile.mkdtemp(prefix="trackA_sdf_neg_")
    incs = list(tree.getroot().iter("include"))

    # N1: 挪第一个锥桶 +0.1 m(跳过 ground_plane/sun 等基础设施 include)
    p1 = os.path.join(tmpdir, "neg1.world")
    t1 = copy.deepcopy(tree)
    inc1 = next(inc for inc in t1.getroot().iter("include")
                if "trackA_cone_" in (inc.findtext("uri") or ""))
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
        if (inc.findtext("uri") or "").strip() == "model://trackA_cone_blue":
            inc.find("uri").text = "model://trackA_cone_red"
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
