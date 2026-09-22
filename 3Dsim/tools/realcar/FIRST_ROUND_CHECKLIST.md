# 第一轮操作清单 —— 只读盘点 + 无动力(2026-09-19 · v2)

> 给队友的**首轮**入口。本轮只做两件事:①**只读盘点**(连上车机,看,不改);②**无动力步骤**
> (部署工具 + 离线单测 + 拉起显示)。**本轮不含任何动力阶段**:不接电调、不发导航目标、不测电机。
> 动力阶段(接电调/发车/导航)在数据回传、我据此确认后再单独授权,不与本轮捆绑。
>
> **本操作卡 v2**:显示步骤(B3)适配**显示 v2 增强包**(对应提交 `9e911ac`),入口改为
> `display_monitor.sh` + `fetch_monitor.bat`/`.sh`。历史压缩包保持原样,本卡不静默覆盖。

---

## A. 只读盘点(只 ssh 查看 + 拉文件,不改车端任何东西)

> 目标:确认「历史假设」哪些还成立,把真实值回传。**任何一项与实际不符就停下回报,不硬猜。**

### A1. 连接与系统
```bash
# 先确认车机当前 IP(勿假设 192.168.5.100;队友自备直连方式,SSH 密钥直连或密码由队友定)
ssh orange-pi@<车机IP> 'echo 已连上; hostname; lsb_release -d 2>/dev/null; uname -r'
# ROS2 发行版(应 Humble)
ssh orange-pi@<车机IP> 'source /opt/ros/*/setup.bash 2>/dev/null; ros2 --version 2>/dev/null || echo "ROS2 未找到"'
# 部署根目录(历史 ~/racecar_team/)
ssh orange-pi@<车机IP> 'ls -d ~/racecar_team 2>/dev/null && ls ~/racecar_team'
```

### A2. 现役运行进程(先看,不动)
```bash
# 有哪些车控/编码器进程在跑?读数前先确认「谁在读串口」——唯一读取者原则,起新读前必须先停旧的
ssh orange-pi@<车机IP> 'ps aux | grep -E "encoder|gate|driver|safety|amcl|nav2|slam" | grep -v grep'
# 串口设备是否存在(历史 /dev/encoder 或 /dev/ttyACM0)
ssh orange-pi@<车机IP> 'ls -l /dev/encoder /dev/ttyACM* 2>/dev/null'
```

### A3. 配置与地图(拉回 + 记 md5,不改)
```bash
# 拉现役 EKF 配置(核对 odom0 到底是 encoder_gate 还是 encoder)
ssh orange-pi@<车机IP> 'cat ~/racecar_team/*.yaml 2>/dev/null | grep -A3 -i odom0 || find ~/racecar_team -name "*.yaml" | head'
# 车端有哪些地图文件(导航演示才需要;显示不需要)
ssh orange-pi@<车机IP> 'find ~/racecar_team ~/racecar -name "*.yaml" -o -name "*.pgm" 2>/dev/null | head -50'
# Python 依赖(显示需要 numpy+matplotlib+rclpy;tap 需要 pyserial)
ssh orange-pi@<车机IP> 'python3 -c "import rclpy, numpy, matplotlib, serial; print(\"依赖齐\")" 2>&1 | tail -1'
```

### A4. 盘点表回传(填完发我)
| # | 项 | 历史假设 | 队友实测 |
|---|---|---|---|
| 1 | 车机 IP / SSH 用户 | `orange-pi@192.168.5.100` | |
| 2 | ROS2 发行版 | Humble | |
| 3 | 部署根目录 | `~/racecar_team/` | |
| 4 | 现役 EKF `odom0` | `encoder_gate` | |
| 5 | 串口设备名 | `/dev/encoder` | |
| 6 | 现役编码器进程 | `encoder_node`/`encoder_gate`(待确认) | |
| 7 | 车端地图文件 | 有 09-17 SLAM 图(待确认) | |
| 8 | Python 依赖 | rclpy/numpy/matplotlib/serial(待确认) | |
| 9 | 电调是否断开 | 应已断开(无动力前提) | |

---

## B. 无动力步骤(部署 + 离线单测 + 显示)

> 全程电调断开、无导航运动源。**只做本节能做的事,不做 §C 的动力项。**

### B1. 部署(备份 → scp → 核验)
见 `README.md` §3。目标目录 `~/racecar_team/encoder_fix/` + `~/racecar_team/`(显示脚本)。
```bash
# 部署后核验 SHA256(以包内 MANIFEST.sha256 为准)
ssh orange-pi@<车机IP> 'cd ~/racecar_team/encoder_fix && sha256sum -c MANIFEST.sha256'
```

### B2. 离线单测(不接串口、不连 ROS)
```bash
ssh orange-pi@<车机IP> 'cd ~/racecar_team/encoder_fix && python3 test_encoder_proto.py && python3 test_gate.py'
# 期望 46/46 + 13/13 PASS。失败 = 文件没传完整,重新 scp 或回报,不继续。
```

### B3. 拉起显示(无动力即可做,传感器上线就行)

> 这是本轮就能拿到的「实时画面」成果,不用等导航。完整说明见 `display/DISPLAY_HANDOFF.md`。
> **唯一推荐入口**:车端 `display_monitor.sh` + 队友端 `fetch_monitor.bat`(Windows)或 `fetch_monitor.sh`(Git Bash/WSL)。
> **不再手动 nohup、不再手动 scp 循环。**

**两包使用顺序(先读清)**:
1. 先解压**全量无动力包**(`hwcart_no_power_delivery_20260919.zip`,SHA `be4b2167…`),它提供基础文件(encoder_fix 工具 + 显示文件)。
2. 再按**显示 v2 增强包**(`hwcart_display_v2_20260919.zip`)的清单,替换/新增显示文件。
3. **无需先跑 v1 再跑 v2**——直接部署 v2 文件即可;历史压缩包保持原样,不覆盖。

**若队友已跑过 v1 显示**:先只读核对旧实例(PID + 完整命令行 + 脚本路径),精确停止该 PID,确认退出后再启动 v2。禁止两个渲染器同时跑。
> **v1 若由手动 nohup 启动, 很可能没有 v2 的 PIDFILE, 不能假定 `display_monitor.sh stop` 能停掉它**——只能按下面"先查后杀"。**全程不得用全局 pkill。**

```bash
# ① 只读核对: 列出 map_monitor.py 进程的 PID + 完整命令行(核对 args 里的脚本路径确实是本目录的
#    map_monitor.py, 不是别的同名进程; 可能有多个, 逐个核对)
ssh orange-pi@<车机IP> 'ps -ww -eo pid,args | grep "[m]ap_monitor.py"'
#   记录上面显示的 PID。

# ② 精确停止该 PID(单个 PID, 不用 pkill -f)
ssh orange-pi@<车机IP> 'kill <核对后的PID>'

# ③ 确认已退出(下面命令无输出 = 已退出)
ssh orange-pi@<车机IP> 'ps -p <PID> -o pid= || echo 已退出'
```

**部署(替换显示文件)**:
```bash
ssh orange-pi@<车机IP> 'cp ~/racecar_team/map_monitor.py ~/racecar_team/map_monitor.py.bak_v1_$(date +%Y%m%d) 2>/dev/null || true'
scp map_monitor.py path_display.py display_monitor.sh save_map_pgm.py orange-pi@<车机IP>:~/racecar_team/
ssh orange-pi@<车机IP> 'chmod +x ~/racecar_team/display_monitor.sh'
```

**启动**:
```bash
# 车端(脚本自带: ROS 环境已加载 / 输出目录存在 / 无已有实例 检查, 并记录本轮 PID)
ssh orange-pi@<车机IP> 'cd ~/racecar_team && ./display_monitor.sh start'
# 队友端(Windows cmd; 默认 192.168.5.100 orange-pi 可省略参数)
fetch_monitor.bat
# 然后 Edge 打开 monitor.html(与 monitor.png 同目录)
```

**现场核对清单(端到端,不凭"脚本语法检查通过")**:
- [ ] **画面更新**:推车时蓝轨迹/绿三角移动,红激光点随环境变化;
- [ ] **停止扫描后 STALE**:人为停 `/scan`(或拔激光供电)后,PNG 左上角 `scan: Ns ago [STALE]` 转红;
- [ ] **拉图失败提示**:断开车机 ssh 后,队友端 fetch 打印「拉图失败,保留旧 monitor.png」且旧图仍在;
- [ ] **启停不影响其他节点**:`display_monitor.sh stop` 后,确认传感器/EKF/导航等其他 ROS 节点仍在,只停了 monitor 这一个进程。

> **边界说明(长期遵守)**:2 秒刷新**仅供监视**,不用于遥控、不用于紧急停车判断——停车只靠安全层
> 与物理断电,不依赖画面是否刷新。

### B4.(可选,仅当队友愿意多走一步)编码器停发失效观察
> 这是 `docs/STAGE_A_Q4_FAILURE_TEST.md` 的完整操作卡,**独立于显示**。若本轮时间紧,
> 可先做 A+B1~B3(盘点+部署+单测+显示),Q4 留到队友确认后再做。**不要为赶进度跳过 Q4 的安全前提。**

---

## C. 本轮不做的动力项(显式列出,不捆绑授权)

以下**全部留到下一轮单独授权**,本清单不含、队友本轮也不要碰:
1. 接电调 / 上电 / 发 `cmd_vel` 任何速度指令。
2. 发导航目标(`NavigateToPose`)或起 Nav2 规划。
3. 电机起步 / 低速标定 / 任何地面移动测试。
4. 切换 EKF 配置或起 gate v2 供 EKF(本轮 gate v2 只随包存放,不启用)。

---

## D. 回传(第一轮结束点)

1. §A4 盘点表填完。
2. 部署核验 `sha256sum -c` 输出 + 车端 46/46、13/13 复跑结果。
3. 显示 B3 现场核对清单四项的结果(画面更新 / 停扫描后 STALE / 拉图失败提示 / 启停不影响其他节点)+ 一张 `monitor.png` 截图(标注「无动力,只读」)。
4. 现场任何临时改动(diff)。

收到后我据此:①确认哪些历史假设仍成立;②决定显示方案是否够用(要不要扩展 amcl_pose/路径渲染);
③判断是否进入下一轮(Q4 / 动力前提)。**动力放行由我/用户明确给出,不在本清单内。**
