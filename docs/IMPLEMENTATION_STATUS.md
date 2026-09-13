# 仿真基础工程状态（2026-09-13）

本交付是阶段 0 的准备与可独立验证的基础模块，不是完整 Gazebo 仿真。未运行 colcon、Gazebo、SLAM 或 Nav2，不宣称实车零改动迁移已实现。

## 已完成

- 四个 ament_python 包的基础目录；racecar_sim 和 racecar_bringup_sim 暂无可启动节点。
- 原始源码接口契约及 SHA-256 溯源清单。
- 只读 Ubuntu / ROS2 / Gazebo 依赖检查脚本。
- CSV 锥桶坐标、显式候选间隙、随机种子生成 SDF 及随机清单。
- 独立罚时事件账本、两圈有效时间公式、停车判定及边界测试。

## 本地验证

在本目录执行（Windows 或 Linux，Python 3.10+，无第三方依赖）：

```sh
python -m unittest discover -s tests -v
python hwcart_sim/src/track_generator/track_generator/generate.py --cones examples/cones.csv --gaps examples/gaps.json --seed 42 --board-thickness 0.02 --output work/prototype.world
python tools/check_environment.py
```

环境检查在 Windows 上应退出 1，表示尚未满足目标环境，并非模块测试失败。在目标 Ubuntu shell 中 source Humble 后再次执行。当前会话无法查询 WSL 发行版，不能断定未安装 Ubuntu。

## 生成器限制

examples 仅为直道测试夹具，不是官方 A/B/C/D。锥桶使用静态长方体代理，尺寸 0.28×0.28×0.63m，不可用于锥桶视觉训练、精确激光截面或撞锥位移判罚。挡板长 0.8m、高 0.5m；厚度是调用者明确传入的建模假设，例子 0.02m 不来自规则。候选间隙必须先由赛道设计者确认相邻关系；当前只检查端点、颜色、距离和重复。没有整体赛道相交与挡板互相碰撞检测。未实现红绿灯、标志牌、车道线、A-D 坐标生成。

## 判罚模块限制

事件由上游检测器输入，不从传感器自动判断。event_id 必须标识一次真实事件，重复消息不会重复罚时；同一个锥桶的多次真实撞击应使用不同事件 ID。折线压线每圈最多罚一次。红灯未停车、闯灯等是否叠加，规则未明确，调用方必须明确裁定后输入，模块不自行推测。地图不清晰与无效也需外部裁定。

停车轮子是否在胶带内侧、停车是否稳定、圈线穿越、红灯触发与等待计时、作弊取消成绩等尚无自动检测器。超时调用只适用于比赛仍在运行时；最终成功停车后不继续调用。红灯最长 10 秒与车辆响应导致的超时问题待裁判口径确认，当前严格执行 >10 秒失败。有效时间按 PDF 字面公式减去两次红灯时长和，不擅自改成按圈加权扣除。Tmax=Tmin 时规则公式未定义，显式报错。

## 下一步验收顺序

1. 在可访问的 Ubuntu 22.04 环境运行环境检查，确认 Humble / Classic 11 与各依赖可用。
2. 测定轴距、轮距、轮径、质量/惯量、舵角极限、PWM 映射及传感器外参，解决源码 TF 与物理高度的冲突。
3. 基于已确认参数实现 XACRO 与阿克曼插件，验收直行、转弯、停车、/clock 和 TF 唯一发布者。
4. 将规则图 3/9 几何解析为有出处的 A-D 坐标；再加入真实锥桶模型与视觉设施。
5. 逐步接入 EKF、SLAM、存图切换、Nav2、视觉与两圈状态机，完成闭环回归。

Git 约定：只本地 commit，不自动 push。未更改原始出厂资产。
