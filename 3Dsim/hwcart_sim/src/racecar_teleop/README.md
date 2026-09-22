# racecar_teleop

手柄遥控 → 标准 `geometry_msgs/msg/Twist` 的独立 ROS2 包,面向室外 ROS 无人车
(四轮阿克曼小车)。**只实现标准速度接口**,不直接驱动厂家实车、不修改主工程、
不接管手动/自动仲裁。

## 状态

- ✅ `mapping.py` 纯数值映射(死区/连续缩放/限幅/阿克曼换算/低速模式/非法输入防护)
- ✅ `state.py` 使能/超时状态机(按住才走、松手停车、超时锁定、先松开再按住;单调时钟)
- ✅ `test/` 单测 46 条,纯 Python + pytest,无需 ROS
- ⏳ `teleop_node.py`(rclpy 节点:订阅 /joy、驱动状态机、20Hz 发布)——需 ROS2 Humble
- ⏳ 模拟 Joy 集成测试、实体手柄验收(无硬件,暂标记"未验证")

## 对外接口(未经沟通不得改变)

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/joy` | `sensor_msgs/msg/Joy` |
| 输出 | `/teleop/cmd_vel` | `geometry_msgs/msg/Twist` |

输出字段:仅 `linear.x`(前向速度 m/s)与 `angular.z`(偏航角速度 rad/s),其余为 0。
正速度=前进,正角速度=左转;`omega = v * tan(delta) / wheelbase`,`v=0` 时 `omega=0`。

## 本地跑单测(无需 ROS)

```bash
cd racecar_teleop
python -m pytest test/test_mapping.py -v
```

## 构建与启动(需 Ubuntu 22.04 + ROS2 Humble,本机未验证)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select racecar_teleop
source install/setup.bash
ros2 launch racecar_teleop teleop.launch.py            # 带 joy 驱动
ros2 launch racecar_teleop teleop.launch.py use_joy:=false   # 只启映射节点,配合模拟 /joy
ros2 topic echo /teleop/cmd_vel
```

## 参数与按键映射

见 `config/generic.yaml`(含单位与正负方向注释)。默认是通用双摇杆示例,
按键/轴索引按具体手柄型号覆盖;`low_speed_button: -1` 可禁用低速模式。

## 已知限制

- 本包崩溃或网络中断时无法保证零指令到达,车辆端必须另有命令超时停车机制。
- 使能键是"保持按住",不是硬件急停。
- 尚未在 Humble 上 `colcon build` 验证(当前开发机为 Windows,无 ROS2)。

## 主开发者接入 `/teleop/cmd_vel` 所需步骤

1. `colcon build` + source 后启动本包。
2. 确认 ros2_control 的 `ackermann_steering_controller` `use_stamped_vel: false`。
3. 把控制器 `reference_unstamped` 话题 remap 到 `/teleop/cmd_vel`(或主工程自行转发)。
