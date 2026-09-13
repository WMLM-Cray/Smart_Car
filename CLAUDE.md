# 项目工作规则(07-仿真系统)

本文件约束所有 AI 助手在本仓库内的工作方式。

## 分工模式

| 角色 | 职责 |
|---|---|
| 人 | 写实施任务书(接口/参数/禁止事项/验收条件);检查仓库改动与测试证据;给通过意见或返工清单 |
| AI(实现) | 读任务书 → 实现代码 → 运行测试 → 把结果与问题写入交付记录 |
| 外部 AI(测试) | 按下方"成果目录清单"对仓库做独立测试验收 |

AI 不替人做验收判断;人不抢 AI 的实现工作。

## 硬性规则

1. **每次阶段性成果后,必须在交付说明里列出成果所在目录/文件的完整清单**
   (Windows 侧 F 盘路径 + WSL 侧 `~/hwcart_sim` 路径双份),供外部 AI(如 GPT)测试。
2. **验收逐项进行,不凭"Gazebo 里出现车模"判完成**。标准顺序:
   构建成功 → 控制器激活 → 直行与前轮转向 → 传感器与 TF → 一键启动。
3. 本地 git commit 正常做;**push 到 GitHub 由用户决策**,AI 不自动 push。
4. 未实测的尺寸/外参统一放 `hwcart_sim/src/racecar_sim/config/params.yaml` 并标注
   `[实测假设]`,不在别处散落硬编码。
5. 阶段 1 的 `/cmd_vel` 是标准 Twist 速度语义;**不宣称已兼容厂家 PWM 接口**
   (`linear.x*100+1500` 那套映射属于实车驱动层,见 `contract.json`)。
6. 实装包以本机核对为准(例:ackermann_steering_controller 2.53.3 已核对参数名
   `front_wheels_names` 等),不照搬老教程参数。

## 路径约定

- Windows(git 主副本):`F:\影音\BaiduSyncdisk\programss\华为小车\07-仿真系统\`
- WSL(构建/运行):`/home/lzx/hwcart_sim/`(git 同源;colcon 工作区在其下 `hwcart_sim/`)
- 同步:F 盘 → WSL 用 tar 复制(不经 GitHub);**禁止在 `/mnt/f` 下跑 colcon build**
