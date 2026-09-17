# -*- coding: utf-8 -*-
"""按主线任务书六条判据,对 racecar_teleop 状态机做独立黑盒复验。

不依赖 ROS,直接驱动纯逻辑层 TeleopStateMachine(该层与 teleop_node.py 的
接线已审查确认:on_joy/on_tick 均传 time.monotonic(),节点不启用 use_sim_time)。

判据出处:2026-09-17 主线任务书第二节"必须复验"1-6 条;
判据7 = 2026-09-17 主线追加(油门回中门:锁定恢复需"松开重按使能 + 油门回死区一次")。
判据3 拆成 3a/3b:
  3a = 断流期间 tick 正常运行(tick 先观察到超时并锁定)——队友已测;
  3b = 恢复消息先于 on_tick 到达(断流窗口内无 tick,任务书字面场景)——
       需 on_joy 消息间隔检查补丁(2026-09-17 引入轮补上),原版必 FAIL。

构造约定(2026-09-17 油门门适配):所有 arm 序列的"松使能"帧一律
`throttle=0.0`(真实松手=松使能与摇杆回中同帧),不再用推杆帧解锁。

运行: python tools/teleop/verify_teleop_task_spec.py [包目录]
  包目录默认 = 仓库内 hwcart_sim/src/racecar_teleop
输出: stdout 逐条 PASS/FAIL + 汇总;退出码 0=全过, 1=有 FAIL
"""
import io
import os
import sys

PKG_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    os.pardir, os.pardir, "hwcart_sim", "src", "racecar_teleop")
sys.path.insert(0, os.path.abspath(PKG_DIR))
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                              errors="replace")

from racecar_teleop.mapping import Parameters          # noqa: E402
from racecar_teleop.state import TeleopStateMachine    # noqa: E402

P = Parameters(
    speed_axis=1, steering_axis=3,
    enable_button=5, low_speed_button=4,
    deadzone=0.08,
    max_speed_forward=0.5, max_speed_reverse=0.3,
    max_steering_angle=0.35, wheelbase=0.36,
    publish_rate=20.0, input_timeout=0.3,
)

def joy(throttle=1.0, steer=0.0, enable=True, low=False):
    """Joy 消息:axes[1]=速度轴(前推=+1 前进), axes[3]=转向轴;buttons[5]=使能."""
    axes = [0.0, throttle, 0.0, steer]
    buttons = [0, 0, 0, 0, 1 if low else 0, 1 if enable else 0]
    return axes, buttons

results = []

def check(cid, desc, ok, detail=""):
    results.append((cid, desc, ok, detail))
    print("%s [%s] %s%s" % ("PASS" if ok else "FAIL", cid, desc,
                            ("  -> " + detail) if detail else ""))

# 判据1: 启动时按住使能,不运动
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(enable=True))
out = sm.on_tick(0.05)
check("1", "启动时按住使能不运动", out == (0.0, 0.0), "输出=%r" % (out,))

# 判据2: 先松开再按使能,才允许运动
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
out = sm.on_tick(0.15)
check("2", "松开后按下才允许运动", out[0] > 0.4 and abs(out[1]) < 1e-9,
      "输出=(%.4f, %.4f)" % out)

# 判据3a: tick观察超时后,按住使能的恢复消息不自行恢复
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
sm.on_tick(0.5)                    # 断流0.4s>0.3s: tick先判超时锁定
out_a = sm.on_tick(0.6)
sm.on_joy(0.61, *joy(enable=True))
out_b = sm.on_tick(0.62)
check("3a", "tick观察超时后,按住使能的恢复消息不自行恢复",
      out_a == (0.0, 0.0) and out_b == (0.0, 0.0),
      "锁定tick=%r 恢复后tick=%r armed=%s" % (out_a, out_b, sm.armed))

# 判据3b: 物理断流后恢复消息先于tick到达,仍输出零(任务书判据3字面场景)
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
sm.on_joy(1.09, *joy(enable=True))  # 断流0.99s,期间一次tick都没执行
out = sm.on_tick(1.10)
check("3b", "物理超时后恢复消息先于tick到达,仍输出零",
      out == (0.0, 0.0),
      "输出=%r armed=%s (非零=漏超时撤销,需on_joy消息间隔检查补丁)" % (out, sm.armed))

# 判据4: 松开并重新按使能后才恢复
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
sm.on_tick(0.5)
sm.on_joy(0.6, *joy(enable=True))
assert sm.on_tick(0.61) == (0.0, 0.0)
sm.on_joy(0.7, *joy(throttle=0.0, enable=False))   # 松手动作含摇杆回中
sm.on_joy(0.8, *joy(enable=True))
out = sm.on_tick(0.85)
check("4", "松开重按使能后恢复运动", out[0] > 0.4, "输出=(%.4f, %.4f)" % out)

# 判据5a: 非法输入 -> 停止并撤销许可
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
sm.on_joy(0.2, [0.0, float("nan"), 0.0, 0.0], joy(enable=True)[1])
out = sm.on_tick(0.25)
check("5a", "NaN轴输入 -> 撤销许可并输出零", out == (0.0, 0.0) and not sm.armed,
      "输出=%r armed=%s" % (out, sm.armed))

# 判据5b: 松使能 -> 立即零
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
sm.on_joy(0.2, *joy(enable=False))
out = sm.on_tick(0.25)
check("5b", "松开使能立即输出零", out == (0.0, 0.0), "输出=%r" % (out,))

# 判据5c: 断流 -> tick 输出零
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
out = sm.on_tick(0.5)
check("5c", "断流(>0.3s)后 tick 输出零", out == (0.0, 0.0), "输出=%r" % (out,))

# 判据6: 超时使用单调时间(代码审查+行为验证)
# 审查结论(2026-09-17): teleop_node.py 两处 time.monotonic() 同源;
# 节点未启用 use_sim_time;state.py 纯逻辑不读时钟,只比较传入 now 序列。
# 行为验证: 单调时钟绝对值任意大不影响判定,只看差值。
sm = TeleopStateMachine(P)
sm.on_joy(1000000.0, *joy(throttle=0.0, enable=False))
sm.on_joy(1000000.05, *joy(enable=True))     # 间隔0.05s: 正常流
out_move = sm.on_tick(1000000.10)
out_timeout = sm.on_tick(1000000.45)         # 距上次消息0.4s>0.3s
check("6", "超时判定只看单调时间差,不依赖绝对值/仿真时钟",
      out_move[0] > 0.4 and out_timeout == (0.0, 0.0),
      "运动tick=(%.2f,..) 停止tick=%r" % (out_move[0], out_timeout))

# 判据7: 油门回中门(2026-09-17 主线追加)——锁定恢复需"松开重按使能 +
# 油门轴回死区一次",防重连后用户仍推着摇杆,一按使能突然起步。
sm = TeleopStateMachine(P)
sm.on_joy(0.0, *joy(throttle=0.0, enable=False))
sm.on_joy(0.1, *joy(enable=True))
assert sm.on_tick(0.15)[0] > 0
sm.on_tick(0.5)                                     # 断流锁定(双门均重置)
sm.on_joy(0.6, *joy(throttle=0.9, enable=False))    # 松使能,但杆仍在死区外
sm.on_joy(0.7, *joy(throttle=0.9, enable=True))     # 重按使能,杆仍推着
out_hold = sm.on_tick(0.75)
sm.on_joy(0.8, *joy(throttle=0.0, enable=True))     # 杆回死区一次(使能可保持)
sm.on_joy(0.9, *joy(throttle=0.9, enable=True))     # 再推杆
out_move = sm.on_tick(0.95)
check("7", "锁定恢复需油门先回死区一次(杆仍推着时按使能不得起步)",
      out_hold == (0.0, 0.0) and out_move[0] > 0.4,
      "杆仍推着tick=%r 回死区后tick=(%.4f, %.4f)" %
      (out_hold, out_move[0], out_move[1]))

n_fail = sum(1 for r in results if not r[2])
print("\n==== 复验汇总: %d 项, PASS=%d FAIL=%d ====" %
      (len(results), len(results) - n_fail, n_fail))
for cid, desc, ok, _d in results:
    print("  %s [%s] %s" % ("PASS" if ok else "FAIL", cid, desc))
sys.exit(1 if n_fail else 0)
