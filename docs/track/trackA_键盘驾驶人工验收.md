# trackA 键盘驾驶人工验收手册(阶段2·可操作演示)

任务书(2026-09-15):启动 A赛道+车辆+RViz(雷达/相机/TF);键盘低速
前进/左右转/倒车/停止;逆时针完整一圈;录短视频并记录碰撞/卡住/出界位置;
保存准确启动步骤。**不改几何、不接 Nav2、不启用未修复的手柄。**

验收日期:________  验收人:________

## 0. 路线总览(先看懂再开)

发车点 (11.0, 0.75) 朝 +x,位于底道(y 0~1.5)中间偏右。逆时针一圈:

```
        ┌────←──── 顶部直道 ────←────┐         ← 顶线约在 y≈5.5(蓝外)
      左端U弯                    顶部黄V窄道          (y≈4.0 红内)
        │      (x≈8.3~13.9,通道宽≈1.4,    │
        ↓        绕 V 形走,减速)          │
        └────→──── 底道直道 ────→────┘
              发车点(11.0,0.75)在此,朝 +x 出发
```

- 右端 U 弯:180° 回头弯,外缘蓝锥、内缘红锥,车道宽 1.5,沿中线走半径≈2.0
- 顶部黄V:中段 x≈8.3~13.9 是 V 形窄道(宽≈1.4,该段无锥桶、以黄带为界),
  这是全程最窄处,降速慢过;**卡住/出界大概率在此,如实记录**
- 左端 U 弯同右端,下来后沿底道向 +x 回到发车点

## 1. 环境自检(WSL,一次)

Windows 侧打开终端(PowerShell/Windows Terminal均可),进入 WSL:

```
wsl -d ubuntu2204-ros2
```

> ⚠️ 不能用默认 `wsl`(默认发行版是 alpine,没有 ROS)。

WSL 内自检(复制整段执行):

```bash
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash
ros2 pkg prefix racecar_test_assets && \
ls ~/hwcart_sim/hwcart_sim/src/racecar_sim/worlds/trackA.world && \
ls ~/hwcart_sim/hwcart_sim/install/racecar_sim/share/racecar_sim/worlds/trackA.world
```

三条都有输出 = 资产包/赛道/构建就绪(9/15 深夜资产接入轮验证时的状态)。
任何一条报错:先执行一次 `colcon build --packages-select racecar_test_assets racecar_sim`
(在 `~/hwcart_sim/hwcart_sim/` 下);仍失败则停下回报,不要盲改。

## 2. 启动(三个 WSL 终端,每个都要先 source)

**约定:每个新终端都执行下面两行**(ROS 环境 + 工作区 + 域隔离):

```bash
source /opt/ros/humble/setup.bash && source ~/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
```

### 终端1:仿真(Gazebo GUI + trackA + 车辆)

```bash
ros2 launch racecar_sim sim_bringup.launch.py \
    world:=trackA.world spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0
```

`use_gui` 默认 true,会弹出 Gazebo 窗口(WSLg)。等待约 35s,成功的标志:

- 终端出现 `Successfully spawned entity [racecar]`
- 两个 spawner 都输出 `active`
- Gazebo 窗口里看到 63 个锥桶围出的赛道和车

Gazebo 视角:滚轮缩放,按住**鼠标中键**拖动平移,左键旋转;想跟车可
菜单 Camera→Follow,选 `racecar`。

### 终端2:RViz(雷达/相机/TF)

```bash
rviz2 -d ~/hwcart_sim/hwcart_sim/src/racecar_sim/rviz/sim.rviz
```

应看到:车模(RobotModel)、TF 箭头族、/scan 点云弧(打到锥桶)、
相机画面(Image,/image_raw)。Fixed frame=odom(手册已知事项)。

### 终端3:键盘遥控

首次需装(一次性,以后跳过):

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

启动:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

启动后**先按 `q` 降速档**(默认 0.5 偏快),建议起步参数:
speed≈0.30、turn≈0.40(按 `q/z` 调速度档、`a/d` 调转向档)。

键位:`i`前进 `k`停 `,`倒车 `j`左转 `l`右转(转向键与 i/, 组合成弧线)。
**焦点必须保持在 teleop 终端**,点到别处键盘就不响应。

## 3. 演示动线与记录表

### 第一部分:基础动作(空旷处,底道中段)

| # | 动作 | 键位 | 通过标准 | 完成 |
|---|---|---|---|---|
| K1 | 低速前进 ~2m | `i` | 平稳起步无抖动 | ☐ |
| K2 | 左转 90° 回正,再右转 90° 回正 | `j`+`i` / `l`+`i` | 转向响应及时,不蹭锥 | ☐ |
| K3 | 倒车 ~1m | `,` | 方向正确无失控 | ☐ |
| K4 | 停车 | `k`(或直接松手,0.5s watchdog 自动零速) | 立即停稳 | ☐ |

### 第二部分:逆时针完整一圈

从发车点出发:底道向 +x → 右端 U 弯(左打)→ 顶部向 -x 过黄V 窄道 →
左端 U 弯(左打)→ 底道回到发车点附近。通过标准:全程未被锥桶/黄带
卡死、未骑出边界;轻微剐蹭记入下表。

一圈记录:

- 完成用时:______; 是否完整闭环回到发车点:☐是 ☐否
- 碰撞/卡住/出界记录(位置尽量给 x,y 或特征物,如"黄V 左臂中段"):

| 时间点 | 位置 | 事件(碰撞/卡住/出界) | 处置(倒车脱困/重发车/…) |
|---|---|---|---|
|  |  |  |  |

### 顺手一条(通道顺序核清数据,只读不改)

一圈跑完后在任一已 source 的终端执行并抄下输出(给"两处核清"补 encoding
权威值):

```bash
ros2 topic echo /image_raw --field encoding --once
```

记录:encoding = ____________(预期 rgb8 或 bgr8,以输出为准)

## 4. 录像

Windows 侧录屏(推荐,能同时录到 Gazebo+RViz+teleop 终端):

- 快捷方式:**Win+Alt+R** 开始/停止录制当前窗口(Xbox Game Bar),或
- OBS(已装则用 OBS,来源选"窗口捕获")

录一段 1~3 分钟:包含起步、基础动作片段、一圈中的黄V 窄道与两个 U 弯、
传感器画面(RViz 点云/相机)至少各露一次。
文件放 `docs/screenshots/trackA_lap.mp4`(>25MB 则放 F 盘别处,只登记路径)。

## 5. 收尾

1. teleop 终端按 `k` 停车 → Ctrl+C 退出 teleop、RViz
2. 终端1 Ctrl+C;若 Gazebo 窗口不退/gzserver 卡死(已知问题):
   Windows 侧 `wsl --shutdown` 兜底(注意会清 WSL /tmp)
3. 录像与记录表归档:

```bash
git add docs/track/trackA_两处核清_20260915.md docs/track/trackA_键盘驾驶人工验收.md
git commit -m "阶段2 trackA 键盘驾驶人工验收:记录与归档"
```

## 已知边界(验收时知悉,不算缺陷)

- 锥桶为队友资产包**静态模型**(static):撞不倒、推不动,撞上=硬挡停车,
  动态锥撞倒/推移为历史对照(872c87d),非当前版本能力
- 发车点 (11.0,0.75) 为 asmp(规则未定发车点)
- 红绿灯模型常亮未接入;黄V 段无锥桶(以黄带为界)
- 相机图像红蓝标签对应关系未定案(见《trackA_两处核清_20260915.md》第二节),
  本轮顺手采 encoding,不影响驾驶

## 本轮成果文件清单

| 文件 | Windows(F盘) | WSL(~/hwcart_sim/) |
|---|---|---|
| 键盘驾驶手册(本文) | `docs/track/trackA_键盘驾驶人工验收.md` | 同相对路径 |
| 两处核清记录 | `docs/track/trackA_两处核清_20260915.md` | 同相对路径 |
