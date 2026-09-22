#!/usr/bin/env bash
# 转向判定调试:pub 转向指令 → 取 joint_states → 逐步展示 awk/python 解析
export ROS2CLI_NO_DAEMON=1
source "$(dirname "$0")/_env.sh"

echo "== 发布转向指令 3s =="
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}" >/dev/null 2>&1
sleep 1

echo "== awk 截取的原始内容(前 25 行) =="
timeout 8 ros2 topic echo /joint_states 2>/dev/null \
  | awk '/^name:/{flag=1} flag{print} /^---$/{if(flag)exit}' > /tmp/js_raw.txt
head -25 /tmp/js_raw.txt
echo "== 文件字节数: $(wc -c < /tmp/js_raw.txt) =="

echo "== python 解析(不吞错误) =="
python3 - <<'PYEOF'
import yaml
with open('/tmp/js_raw.txt') as f:
    d = yaml.safe_load(f)
print('解析结果类型:', type(d))
if d:
    print('keys:', list(d.keys()))
    for n, p in zip(d.get('name', []), d.get('position', [])):
        print(f'  {n}: {p}')
PYEOF
