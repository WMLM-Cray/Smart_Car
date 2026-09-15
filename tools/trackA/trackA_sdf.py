#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""赛道A 锥桶 SDF 生成器(冻结 CSV → Gazebo Classic 场景)。

单源原则:锥桶坐标唯一来源 = docs/track/trackA_cones.csv(trackA_geometry.py
的冻结输出)。本文件不做任何几何决策,只做"CSV → SDF 资产"的忠实转换;
黄V禁放带无锥=CSV 里本来就没有(生成器侧已保证,检查器侧对账兜底)。

产出(全部可重建,不入手工编辑):
  hwcart_sim/src/racecar_sim/models/trackA_cone_{blue,red}/
      model.config + model.sdf + meshes/traffic_cone.dae
  hwcart_sim/src/racecar_sim/worlds/trackA.world
      (ground+sun+state插件 与 racecar.world 同基础;63 个 <include>;
       不覆盖 racecar.world——那是阶段1车辆验收空场)

锥桶尺寸 = 官方值(比赛规则§3.4 道路设置,规则PDF第8页:"锥桶规格:底径280mm,
高630mm",三视图标注 630/280/280):
  锥高 0.63 m / 锥底半径 0.14 m / 底座 0.28×0.28 m(官方三数全覆盖)
  底座厚 0.03 m / 质量 1.0 kg = asmp(规则只给三个数,厚度质量未给)
  collision = 单圆柱 r=0.14 h=0.63(=底径/2,凸近似;方座四角在柱外,保守)
  static=false(动态体,低速碰撞可撞倒——撞桶表现验收前提)
  颜色染在 visual material(diffuse),一 mesh 两色复用;mesh 自身无材质

队友模型替换约定:include pose 即坐标,换锥桶 mesh 只改 model.sdf 的
visual,meshes/ 与 63 处 include pose 不动——"模型替换保持坐标不变"。

用法(WSL):
  python3 tools/trackA/trackA_sdf.py [--csv docs/track/trackA_cones.csv]
"""
import argparse
import csv
import math
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, '..', '..', 'hwcart_sim',
                                    'src', 'racecar_sim'))

# ---- 锥桶尺寸(官方值:规则§3.4"底径280mm,高630mm";残余 asmp 项单独标注) ----
CONE_H = 0.63        # 锥高(官方 630mm)
CONE_R = 0.14        # 锥底半径(官方底径 280mm / 2)
BASE_W = 0.28        # 底座边长(官方 280×280mm)
BASE_H = 0.03        # 底座厚 asmp(规则未给,取常见实锥薄座)
CONE_MASS = 1.0      # 质量 asmp(规则未给,常见 630 锥 PVC 约 1 kg)
COL_R = 0.14         # collision 圆柱半径(=底径/2;方座四角 0.198 在柱外,偏保守)
N_SIDES = 16         # 锥 mesh 棱数
COLORS = {
    "blue": (0.15, 0.40, 0.90, 1.0),
    "red":  (0.90, 0.15, 0.15, 1.0),
}


def gen_cone_dae():
    """16 棱锥 Collada(底圆 z=0,尖 z=CONE_H);flat 法线,无材质(SDF 染色)。"""
    verts, normals, tris = [], [], []

    def tri(a, b, c):
        """按 (a,b,c) 序输出三角形,法线 = (b-a)×(c-a) 归一化。"""
        u = [b[k]-a[k] for k in range(3)]
        v = [c[k]-a[k] for k in range(3)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        ln = math.sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]) or 1.0
        n = tuple(x/ln for x in n)
        base = len(verts)
        for p in (a, b, c):
            verts.append(p)
            normals.append(n)
        tris.append((base, base+1, base+2))

    ring = [(CONE_R*math.cos(2*math.pi*i/N_SIDES),
             CONE_R*math.sin(2*math.pi*i/N_SIDES), 0.0) for i in range(N_SIDES)]
    apex = (0.0, 0.0, CONE_H)
    center = (0.0, 0.0, 0.0)
    for i in range(N_SIDES):
        tri(ring[i], ring[(i+1) % N_SIDES], apex)      # 侧面,法线朝外
        tri(ring[i], ring[(i+1) % N_SIDES], center)    # 底面,法线朝下
    pos = " ".join("%.6f %.6f %.6f" % v for v in verts)
    nrm = " ".join("%.6f %.6f %.6f" % v for v in normals)
    idx = " ".join("%d %d %d" % t for t in tris)
    return """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>trackA_sdf.py</authoring_tool></contributor>
    <created>2026-09-15T00:00:00Z</created>
    <modified>2026-09-15T00:00:00Z</modified>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="traffic_cone_geom" name="traffic_cone">
      <mesh>
        <source id="positions">
          <float_array id="positions-array" count="{nv}">{pos}</float_array>
          <technique_common>
            <accessor source="#positions-array" count="{nv}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="normals">
          <float_array id="normals-array" count="{nv}">{nrm}</float_array>
          <technique_common>
            <accessor source="#normals-array" count="{nv}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="verts">
          <input semantic="POSITION" source="#positions"/>
          <input semantic="NORMAL" source="#normals"/>
        </vertices>
        <triangles count="{nt}">
          <input semantic="VERTEX" source="#verts" offset="0"/>
          <p>{idx}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="cone_node" name="cone_node" type="NODE">
        <instance_geometry url="#traffic_cone_geom"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
""".format(nv=len(verts)*3, pos=pos, nrm=nrm, nt=len(tris), idx=idx)


def gen_model_dir(color, dae_text):
    """trackA_cone_{color} model 目录(颜色只染 material,几何两色同构)。"""
    d = os.path.join(PKG, 'models', 'trackA_cone_%s' % color)
    mesh_dir = os.path.join(d, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    with open(os.path.join(mesh_dir, 'traffic_cone.dae'), 'w',
              encoding='utf-8', newline='\n') as f:
        f.write(dae_text)
    r, g, b, a = COLORS[color]
    with open(os.path.join(d, 'model.config'), 'w',
              encoding='utf-8', newline='\n') as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>trackA_cone_{c}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>TrackA traffic cone ({c}) - coords owned by frozen CSV, see
tools/trackA/trackA_geometry.py; height/base = official rule 3.4 (630mm
height, 280mm bottom); base thickness &amp; mass are simulation assumptions.
Generated by tools/trackA/trackA_sdf.py - do not edit poses here.</description>
</model>
""".format(c=color))
    with open(os.path.join(d, 'model.sdf'), 'w',
              encoding='utf-8', newline='\n') as f:
        f.write("""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="trackA_cone_{c}">
    <!-- 动态体:低速碰撞须能撞倒/撞动(撞桶表现验收前提) -->
    <static>false</static>
    <link name="cone">
      <inertial>
        <!-- 质量 1.0 kg asmp;惯量按等参圆柱 m(3r^2+h^2)/12, mr^2/2 实算 -->
        <mass>{m}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <!-- visual: 底座 box + 16棱锥 mesh(高/底径=官方 630/280mm) -->
      <visual name="base">
        <pose>0 0 {bzh} 0 0 0</pose>
        <geometry><box><size>{bw} {bw} {bh}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
      <visual name="cone">
        <pose>0 0 0 0 0 0</pose>
        <geometry><mesh><uri>model://trackA_cone_{c}/meshes/traffic_cone.dae</uri></mesh></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
      <!-- collision: 单圆柱 r=底径/2 h=锥高(官方 280/630mm;方座四角在柱外,保守) -->
      <collision name="body">
        <pose>0 0 {czh} 0 0 0</pose>
        <geometry><cylinder><radius>{cr}</radius><length>{ch}</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
""".format(c=color, r=r, g=g, b=b, a=a,
           m=CONE_MASS,
           ixx=CONE_MASS*(3*COL_R*COL_R + CONE_H*CONE_H)/12.0,
           iyy=CONE_MASS*(3*COL_R*COL_R + CONE_H*CONE_H)/12.0,
           izz=CONE_MASS*COL_R*COL_R/2.0,
           bw=BASE_W, bh=BASE_H, bzh=BASE_H/2.0,
           cr=COL_R, ch=CONE_H, czh=CONE_H/2.0))
    return d


def gen_world(cones):
    """trackA.world:与 racecar.world 同基础(ground+sun+state 插件)+ 63 include。
    include pose = CSV (x, y) 直接使用(赛道坐标系与 Gazebo world 同 x右/y上,
    原点=赛道外边界左下角;z=0 贴地)。"""
    inc = []
    for c in cones:
        inc.append("    <include>")
        inc.append("      <uri>model://trackA_cone_%s</uri>" % c["color"])
        inc.append("      <name>cone_%s</name>" % c["id"])
        inc.append("      <pose>%.6f %.6f 0 0 0 0</pose>" % (c["x"], c["y"]))
        inc.append("    </include>")
    world = """<?xml version="1.0"?>
<!-- TrackA cone field - generated by tools/trackA/trackA_sdf.py from the
     frozen CSV (docs/track/trackA_cones.csv). Do not edit poses by hand:
     regenerate instead. Base physics/scene mirrors worlds/racecar.world;
     vehicle is NOT spawned here (added by bringup in a later step). -->
<sdf version="1.6">
  <world name="trackA">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
{inc}
  </world>
</sdf>
""".format(inc="\n".join(inc))
    path = os.path.join(PKG, 'worlds', 'trackA.world')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(world)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.normpath(os.path.join(
        HERE, '..', '..', 'docs', 'track', 'trackA_cones.csv')))
    a = ap.parse_args()
    with open(a.csv, encoding='utf-8', newline='') as f:
        cones = [{"id": r["id"].strip(), "color": r["color"].strip(),
                  "x": float(r["x"]), "y": float(r["y"])}
                 for r in csv.DictReader(f)]
    dae = gen_cone_dae()
    dirs = [gen_model_dir(c, dae) for c in ("blue", "red")]
    world = gen_world(cones)
    nb = sum(1 for c in cones if c["color"] == "blue")
    nr = len(cones) - nb
    print("cones from CSV: %d total (%d blue + %d red)" % (len(cones), nb, nr))
    for d in dirs:
        print("model -> %s" % d)
    print("world -> %s" % world)
    print("cone size: h=%.2f bottom_r=%.2f base=%.2f (official rule 3.4: 630/280/280mm;"
          " thickness/mass asmp, dynamic static=false)"
          % (CONE_H, CONE_R, BASE_W))


if __name__ == "__main__":
    main()
