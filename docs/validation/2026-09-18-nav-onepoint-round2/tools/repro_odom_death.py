#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""repro_odom_death.py —— "里程计失效后如何真正停住" 的离线行为复现(2026-09-18,纯离线)

目的: 把"odom 停但运动指令非零"这一故障形态,按实际链路逐层推演,回答一个问题——
当编码器(odom0 唯一纵向供血源)在运动中死掉时,现有链路里到底有没有任何一层会让车停住。

结论先行(与 AUDIT_318b3d8 §4 行11、EVIDENCE_TIMELINE 一致):
  - 编码器串口死 = encoder_node 的 timer 回调阻塞在 ser.read(7)(现役节点 Serial 无 timeout,
    源码行100),/encoder 不再有新消息(变 stale),**不是发零**;
  - gate v1 只在收到 /encoder 时才转发,/encoder 停 -> /encoder_gate 也停(stale);
  - EKF 到 odom0 sensor_timeout=2.0s 才把 encoder_gate 从融合剔除,但**仍按 20Hz 持续发
    odom_combined**,且速度分量 vx 变"纯预测"(process_noise_covariance=1e-5 极小 -> 不衰减,
    停在最后融合值),位置继续积分 = 静默漂移;
  - AMCL 拿漂移的 odom 继续跑运动模型 -> 位姿错但不知;controller 按 AMCL 位姿判断"没到目标"
    -> 继续发 0.55;/cmd_vel 一直新鲜;
  - cmd_vel_safety 只看 /cmd_vel 新鲜度(0.5s),/cmd_vel 新鲜 -> 一直放行非零;
  - 结论: 现有链路**没有任何一层会让车停住**,唯一可靠停 = 物理断电源。

本脚本是**行为模型推演**(非 robot_localization 重实现),数值用 [配置]/[源码] 推断,不冒充实测。
"""
import sys

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

# 参数(来源标注在注释里,均 [配置]/[源码] 推断)
ENC_HZ = 50.0            # encoder_node create_timer(0.02) -> 50Hz [源码]
EKF_HZ = 20.0            # ekf.yaml frequency: 20.0 [配置]
EKF_ODOM0_TIMEOUT = 2.0  # ekf.yaml sensor_timeout: 2.0 [配置]
VX_LAST = 0.55           # 失效前最后融合的纵向速度(假想巡航值)
SAFETY_CMD_TIMEOUT = 0.5  # cmd_vel_safety cmd_timeout [源码]
DEATH_T = 10.0           # 编码器串口在此刻死


def sim():
    print("=== 复现: 编码器在运动中死掉(odom 停 vs 指令非零) ===\n")
    print("链路: encoder(/encoder) -> gate(/encoder_gate) -> EKF(odom_combined)"
          " -> AMCL -> controller(/cmd_vel) -> cmd_vel_safety(/car_cmd_vel)\n")

    # 状态
    encoder_alive = True
    gate_stale = False          # /encoder_gate 是否 stale
    ekf_odom0_in = True         # EKF 是否还在融合 odom0
    vx = VX_LAST                # EKF 报告的 vx(最后融合值 / 之后纯预测保持)
    x = 0.0                     # EKF 报告的位置 x(积分)
    cmd = 0.55                  # controller 持续下发(盲跑)
    safety_out = 0.55           # 安全层放行值

    # 时间线,每 0.5s 一档
    print(f"{'t(s)':>5} | {'encoder':^7} {'gate':^6} {'EKF(odom0)':^10} "
          f"{'odom.stamp':^10} {'odom.vx':^8} {'cmd_vel':^8} {'safety→car':^11} | 说明")
    print("-" * 96)

    t = 0.0
    step = 0.5
    while t <= DEATH_T + 5.0:
        if t >= DEATH_T:
            encoder_alive = False
            gate_stale = True
        # EKF 剔除 odom0 的时机:sensor_timeout 后
        if t >= DEATH_T + EKF_ODOM0_TIMEOUT:
            ekf_odom0_in = False

        enc = "50Hz" if encoder_alive else "停(stale)"
        gate = "转发" if not gate_stale else "停(stale)"
        ekf = "融合中" if ekf_odom0_in else "已剔除"
        stamp = "推进"       # EKF 无论有无 odom0 都按 20Hz 发,stamp 恒推进
        if not ekf_odom0_in:
            # 纯预测:vx 停在最后值不衰减,位置继续积分 = 漂移
            vx = VX_LAST
        x += vx * step

        # cmd_vel:controller 按 AMCL(漂移的 odom)判断没到目标,继续发 0.55
        # safety:只看 /cmd_vel 新鲜度(0.5s),新鲜 -> 放行非零
        safety_out = cmd

        desc = ""
        if t < DEATH_T:
            desc = "健康:编码器供血,EKF 融合,车按指令走"
        elif t < DEATH_T + EKF_ODOM0_TIMEOUT:
            desc = "编码器已死,gate/EKF odom0 尚未剔除(stale 累积中)"
        elif t == DEATH_T + EKF_ODOM0_TIMEOUT:
            desc = "← 此刻 EKF 剔除 odom0,但 odom 仍发、vx 保持、位置漂移"
        else:
            desc = "odom 漂移中,AMCL 被骗,controller 继续 0.55,**无任何层停车**"

        print(f"{t:>5.1f} | {enc:^7} {gate:^6} {ekf:^10} {stamp:^10} "
              f"{vx:^8.2f} {cmd:^8.2f} {safety_out:^11.2f} | {desc}")
        t += step

    print("\n=== 三个信号源在失效后的表现(关键区分) ===\n")
    print("1) 编码器新鲜度 /encoder_gate 最后消息年龄:")
    print("   失效即变 stale(age 持续增长) -> **能直接感知'物理里程计源死了'**。\n")
    print("2) EKF 输出时间戳 odom_combined.header.stamp:")
    print("   失效后仍按 20Hz 推进(age 恒≈0) -> **假健康信号**,不能作为'里程计健康'依据。\n")
    print("3) 定位有效性 AMCL(协方差/粒子扩散/score):")
    print("   会随漂移劣化,但 AMCL 不自断,是慢噪声信号 -> 只宜作后验复核,不作触发。\n")

    print("=== 结论 ===")
    print("现有 cmd_vel_safety 唯一的停车条件 = /cmd_vel 超时(0.5s),但盲跑的 nav2 会持续发 /cmd_vel,")
    print("因此**编码器死不会触发任何停车**。要真正停住,需在安全层加'odom 源新鲜度'监控")
    print("(详见 ODOM_DEATH_STOP_DESIGN.md),复用 publish_zero 置零路径,且恢复时不自动重发旧指令。")


if __name__ == '__main__':
    sim()
