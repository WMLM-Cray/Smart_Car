# 单点导航第二轮 (2026-09-18 晚) — 发车成功/碰撞/根案/encoder_gate 修复

**状态:发车两轮。第一轮车未动(假位姿致死),第二轮车真实移动并碰撞实物(无损失)。
根因定案=编码器毛刺→假odom→AMCL位姿偏移→路径在现实偏移。encoder_gate 闸门已部署验证,
差最后一步(推车验证+带闸门发车)。**

## 一、执行时间线(对照 0917 七步路线)

| 步 | 结果 | 备注 |
|---|---|---|
| 1 链健康 | ✅ | 车机刚上电=SHM净,节点全起,scan_filter 12Hz |
| 2 initpose+推车收敛 | ✅ | 候选(0.010,-0.240,166.5°)生效,推车~1m(实测0.654m,方向与新车头向差1.6°自洽),cov 0.09→0.0029 |
| 3 costmap 复查 | ✅ | **雪花消失**(墙线+大片自由);左侧0.33m有实物障碍线(贯车头尾,膨胀圈渗到0.19m,未侵入footprint) |
| 4 规划验证 | ✅ | GridBased 出路径,controller 持续发速(无动力下 Failed to make progress=预期) |
| 5 限幅部署 | ✅ | 方案B:desired_linear_vel 0.10→0.55,启动传参 -p max_linear:=0.55,MD5对账一致 |
| 6 发车① | ⚠️车未动 | 电调接通瞬间毛刺→AMCL跳到(-1.092,-0.173)→起点lethal→从未发速度。用户证词"车没动"与日志自洽 |
| 7 initpose拉回+发车② | ✅车动了/**碰了** | 车真实移动0.5m+撞上障碍,用户搬回。假odom又叠0.5m纵向偏移→路径偏移撞实物 |

## 二、核心成果

1. **速度链真实贯通**(今日主目标达成): 0.55过电调死区(485mm/s)→driver→轮子转。
   伴随验证: 安全层超时零速兜底(碰撞后 driver 回 1540/1500 中位)。
2. **碰撞根因定案**(三轮证据链):
   - 厂家 encoder_node DueVelData = 无校验三字节状态机(帧头01 03 02+2字节,无CRC无钳幅)
   - 串口错位(电调接通/cmd_vel流量启动时)→解出任意速度→EKF(odom0:/encoder 速度积分)产出
     0.47m/1.1m/0.5m+三次假位移
   - AMCL位姿被拖偏0.5m→路径避的是地图虚拟障碍→现实正撞实物
3. **encoder_gate 闸门部署**(厂家文件零改动):
   - /encoder → gate(|v|>1.2 丢帧 + |Δv|>0.35/帧 丢帧,触发log) → /encoder_gate → EKF
   - ekf.yaml odom0 切源(备份 .bak_pregate0918)
   - 验证: odom_combined TF 从(0,0)恢复发布,静止恒定无漂移
   - 门限依据: 限幅0.55+余量=1.2; 真车加速度≤1.3m/s²(EKF配置)帧间Δv≤0.1不误拦;
     撞墙堵转瞬跳/毛刺被拦(堵转轮速本不代表车体)
4. 重定位对齐: 新候选 (0.250,-0.140,172.5°) score 0.614(top5稳聚簇,优于昨晚0.565),
   initpose 已生效(首帧微调3cm)。**未推车验证**——明日第一步。

## 三、今日新坑(全部实证)

1. **车机钟向后跳变**: 起链5.4min后发生(某校时服务,timedatectl显示synchronized:yes但NTP inactive)。
   TF缓存留"未来戳"→激光帧全比缓存旧→Message Filter dropping 3条→**10s后自愈**(缓存窗口滑过)。
   处置: 见 dropping 刷屏先等10s,别急着重启链。
2. **pkill -f 自杀新机制**: 字符类[p]attern只防"模式匹配模式自身",**不防自己命令串其他部分
   含目标字面**(如路径 ~/racecar_team/encoder_gate.py 含 encoder_gate 字面)。两次 exit 255 实证。
   处置: 命令串里避免出现目标字面(变量拼接/字符类拆所有出现处),或用 PID 精确杀。
3. **fastrtps SHM 坏段(本轮变体)**: 只拦**新起**的 CLI/进程(open_and_lock_file failed),
   已运行主链不受影响,RTPS 降级后新进程也能工作(gate 带着报错正常收发)。不用重启车机。
4. **Odometry 消息双层路径**: m.twist.twist.linear.x(twist=TwistWithCovariance 内还有 .twist)。
5. **launch 死后孤儿**: pkill Run_car.launch 后 ekf/lslidar 必脱组存活,**每次都要按 PID 补杀**
   (本轮杀了两回,PID 2999/3003 → 4805/4809)。
6. **Nav2 "lethal" 语义**: planner 的 Starting point in lethal space 判定阈值**含 inscribed 99**
   (膨胀圈也卡起点),不止真 lethal 100。
7. 探针自坑: 等待循环提前退出(数据早到)→can_transform 干等期间无spin→TF buffer饿死。
   处置: 固定 spin 满时长(12s),数据到了继续喂 TF(DDS发现~4s)。
8. echo 假象再+1: amcl_pose --once 多轮同值秒回,不可作"在更新"证据;探针新帧计数才权威。

## 四、明日路线(建议顺序)

1. 起链(safety_nav_one.sh 已含 gate+限幅0.55) + scan_filter
2. initpose: 先采帧对齐(车若被动过),原位则 (0.250,-0.140,172.5°)
3. **推车验证**: AMCL位移对账 ±10% + encoder_gate_last.log 查丢帧计数
   (慢推 v<0.35/帧 应全放行;若误拦→放宽 DV_MAX)
4. probe_now_cost 查起点(左侧障碍膨胀圈是否贴近)
5. 发车(目标避左侧,电调断2s再通唤醒): 全程盯 encoder_gate_last.log——
   **毛刺若来应见"丢帧"日志而不是 odom 跳变**,这是 gate 生效的直接证据
6. 顺利→1m/2m 多目标;仍跳→gate 门限失效,转真修 DueVelData(加帧校验,需厂家帧格式资料)
7. 归档提交

## 五、工具与文件变动

- 新增 tools/realcar/encoder_gate.py(车机 ~/racecar_team/ 同步)
- 修改 tools/realcar/safety_nav_one.sh(挂 gate,terminate 同收)
- 修改 tools/realcar/nav_safe.yaml(desired 0.55;车机有 .bak_scanfilter/.bak_pre0918 前史)
- 车机 /home/orange-pi/racecar/install/.../ekf.yaml odom0 切 encoder_gate(备份 .bak_pregate0918)
- 新探针(本目录 tools/): probe_now_cost.py(实时TF位姿代价+障碍格车系定位),
  probe_drift.py(odom/amcl 静止漂移观测), probe_tf_time.py(TF时戳体检)
- 旧探针注意: probe_costmap.py / probe_ascii.py 的 S/G 标记是 0917 旧候选硬编码,
  只作参考线,**不代表当前位姿**(本轮差点因此误判)
