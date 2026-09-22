#!/bin/bash
# [Nav2调试终极探针] 左U顶段(TL弧R1.0)不可规划问题:
#   ①转储 /global_costmap/costmap(规划器实际所见,含膨胀253带)→渲染PNG+数值
#   ②沿左U自下而上逐点打 /compute_path_to_pose,找出可达边界
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim

pkill -INT -f "navigation_bringup.launch.py" 2>/dev/null; sleep 3
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null; sleep 1
Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb6.log 2>&1 &
XV=$!
sleep 2
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim navigation_bringup.launch.py \
    use_gui:=false world:=trackA.world spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    nav_rviz:=false >/tmp/mega_probe.log 2>&1 &
OK=0
for i in $(seq 1 120); do
  N=$(grep -c "Managed nodes are active" /tmp/mega_probe.log 2>/dev/null || echo 0)
  [ "$N" -ge 2 ] && { OK=1; echo "第$((i*2))s:双管理器激活"; break; }
  sleep 2
done
[ "$OK" = "1" ] || { echo "激活失败"; grep -E "Failed|Aborting" /tmp/mega_probe.log|head -4; \
  pkill -INT -f navigation_bringup.launch.py; sleep 4; pkill -9 -f gzserver; kill $XV; exit 1; }
sleep 8

echo "=== ① 转储全局代价地图 ==="
timeout 25 ros2 topic echo --once /global_costmap/costmap > /tmp/gcm.txt 2>/dev/null
wc -c /tmp/gcm.txt

python3 - << 'PYEOF'
import yaml, numpy as np
m = yaml.safe_load(open('/tmp/gcm.txt'))
md = m['metadata']; res = md['resolution']
ox, oy = md['origin']['position']['x'], md['origin']['position']['y']
w, h = md['size_x'], md['size_y']
data = np.array(m['data'], dtype=np.uint8).reshape(h, w)
print(f"代价地图 {w}x{h}@{res} origin=({ox:.2f},{oy:.2f})  "
      f"253+格={int((data>=253).sum())}  200-252格={int(((data>=200)&(data<253)).sum())}")
np.save('/tmp/gcm.npy', data)
# 关键位姿的代价 + 距253带距离场
lethal = data >= 253
INF = 1e9; dist = np.full((h, w), INF); dist[lethal] = 0.0
fr = list(zip(*np.nonzero(lethal)))
st = ((1,0,res),(-1,0,res),(0,1,res),(0,-1,res),
      (1,1,res*1.414),(1,-1,res*1.414),(-1,1,res*1.414),(-1,-1,res*1.414))
while fr:
    nx = []
    for r, c in fr:
        for dr, dc, cs in st:
            nr, nc = r+dr, c+dc
            if 0 <= nr < h and 0 <= nc < w and dist[r,c]+cs < dist[nr,nc]:
                dist[nr,nc] = dist[r,c]+cs; nx.append((nr,nc))
    fr = nx
def cell(x, y):
    j = int((y-oy)/res); i = int((x-ox)/res)
    return h-1-j, i
def probe(x, y):
    r, c = cell(x, y)
    if not (0 <= r < h and 0 <= c < w): return "越界"
    return f"cost={data[r,c]:3d} 距253带={dist[r,c]:.3f}m"
print("\n== 左U顶段(TL弧)沿线位姿 cost/距253带 ==")
for x, y, tag in [(-2.60,2.00,'P4成功点'),(-2.60,3.00,'BL→TL过渡'),(-2.58,3.60,'TL下段'),
                  (-2.50,4.00,'TL中段'),(-2.40,4.30,'TL上段'),(-2.30,4.60,'TL出口'),
                  (-2.30,4.75,'P1目标'),(-2.10,4.74,'近G4起点'),(-1.89,4.74,'G4起点'),
                  (-2.00,4.75,'G3目标'),(-1.60,4.75,'顶道')]:
    print(f"  ({x:6.2f},{y:5.2f}) {tag}: {probe(x,y)}")
# TL弧区每行最宽253自由带
print("\n== 左U区(x∈[-2.75,-1.55])逐y的253自由宽度 ==")
c0, c1 = int((-2.75-ox)/res), int((-1.55-ox)/res)
for yy in np.arange(0.3, 5.3, 0.5):
    r, _ = cell(0, yy)
    row = lethal[r, c0:c1]
    free = np.where(~row)[0]
    if free.size:
        print(f"  y={yy:4.1f}: 253自由带 [{ox+(c0+free.min())*res:.2f},{ox+(c0+free.max())*res:.2f}] 宽{(free.max()-free.min())*res:.2f}m")
    else:
        print(f"  y={yy:4.1f}: 全列253封锁!")
# 渲染:灰=cost,红=253+,黄=目标点
S=4; H, W = h*S, w*S
rgb = np.zeros((H, W, 3), np.uint8)
col = np.stack([data, data, data], axis=-1).repeat(S, 0).repeat(S, 1)
col[..., 0] = np.clip(col[..., 0].astype(int) + (data >= 253).astype(int)*200, 0, 255)
rgb = col
for x, y in [(-2.6,2.0),(-2.3,4.75),(0,0),(-1.89,4.74),(2.0,4.75),(-2.0,0.0)]:
    r, c = cell(x, y)
    if 0 <= r < h and 0 <= c < w:
        rgb[max(0,r*S-8):(r+1)*S+8, max(0,c*S-8):(c+1)*S+8] = (255, 255, 0)
open('/tmp/gcm_overlay.ppm', 'wb').write(f"P6\n{W} {H}\n255\n".encode() + rgb.tobytes())
PYEOF
convert /tmp/gcm_overlay.ppm /tmp/gcm_overlay.png 2>/dev/null \
  && cp /tmp/gcm_overlay.png docs/track/gcm_overlay_debug.png && echo "PNG已存 docs/track/gcm_overlay_debug.png"

yaw2quat() { python3 -c "import math,sys;a=math.radians(float(sys.argv[1]));print(f'{math.sin(a/2):.6f} {math.cos(a/2):.6f}')" "$1"; }
try_plan() {
  local NM="$1" X="$2" Y="$3" YAW="$4"
  read Z W <<< "$(yaw2quat "$YAW")"
  R=$(timeout 40 ros2 action send_goal /compute_path_to_pose \
    nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $Z, w: $W}}}, planner_id: 'GridBased'}" 2>&1 \
    | grep -oE "SUCCEEDED|ABORTED" | head -1)
  echo "  $NM goal($X,$Y,${YAW}°): ${R:-无响应}"
}
echo "=== ② 左U自下而上可达边界二分 ==="
try_plan "A 过渡"  -2.60 3.00 90
try_plan "B TL下"  -2.58 3.60 90
try_plan "C TL中"  -2.50 4.00 60
try_plan "D TL上"  -2.40 4.30 30
try_plan "E TL出口" -2.32 4.55 10
try_plan "F 顶道"  -2.10 4.70 0
try_plan "G1' 顶道东" -1.80 4.74 0
try_plan "H yaw敏感" -2.30 4.60 350

kill -INT $(pgrep -f navigation_bringup.launch.py) 2>/dev/null; sleep 4
pkill -9 -f gzserver 2>/dev/null; kill $XV 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
cp /tmp/mega_probe.log docs/track/logs/mega_probe.log 2>/dev/null
echo "终极探针结束"
