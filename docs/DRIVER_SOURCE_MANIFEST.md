# Driver 源码 / 安装文件 / 运行进程 对照表（2026-09-19，待队友回传填入）

> 目的：把「本机 vendor_src 源码」与「车端 install 二进制 / src 源码 / 运行进程」逐项对账，
> 坐实 driver 公式偏置（1500 vs 1540）与订阅话题（node vs node_one）的**运行时真值**。
> 本表只列本地已确认事实 + 留空待队友回传；**不要求队友为填表新增任何运动或起新进程**，
> 全部是只读核对（读文件 / ps / md5 / strings）。

---

## 一、driver 源码清单（本地 vendor_src，出厂 tar 版）

仓库路径根：`work/vendor_src/racecar/src/racecar_driver/`

| # | 文件 | 本地路径（相对仓库根） | 大小 | 作用 | 关键内容 |
|---|---|---|---|---|---|
| 1 | `racecar_driver_node.cpp` | `work/vendor_src/racecar/src/racecar_driver/src/` | 2495 B | 驱动节点，订 `/car_cmd_vel` + `/teleop_cmd_vel` | 公式 `linear_x = linear.x*100 + 1500`（cpp:45）；节点名 `racecar_driver`（cpp:14） |
| 2 | `racecar_driver_node_one.cpp` | 同上 | 2495 B | 驱动节点，订 `/cmd_vel` + `/teleop_cmd_vel` | 与 #1 逐字相同，唯一区别订阅话题（#1 的 33 行注释 ↔ #2 的 32 行） |
| 3 | `racecar_driver.h` | `.../include/` | 423 B | 串口 API 声明 | `Open_Serial_Dev` / `art_racecar_init(speed,dev)` / `send_cmd(motor_pwm,servo_pwm)` |
| 4 | `libracecar_driver.a` | `.../lib/` | 8382 B | 预编译静态库（串口实现） | 帧格式/校验/断流行为**源码不可见**（REALCAR_CONTROL_AUDIT §2.4） |
| 5 | `CMakeLists.txt` | `.../`（根） | 1002 B | 构建 | 编译 `racecar_driver_node` + `racecar_driver_node_one`，链接 #4，`-no-pie`，install 到 `lib/racecar_driver/` |
| 6 | `package.xml` | `.../`（根） | 522 B | 包元数据 | `racecar_driver`，依赖 rclcpp + geometry_msgs |
| 7 | `README` | `.../`（根） | 469 B | 说明 | PWM 合法范围 500–2500 µs |

**本地源码 sha256 指纹（队友可比对车端 src 是否同源）：**

| 文件 | sha256（本地 vendor_src） |
|---|---|
| `src/racecar_driver_node.cpp` | `3cb58a4b8d4c7613bc7e3fef279466265bdc6542d8b775a7a85615262dd9928e` |
| `src/racecar_driver_node_one.cpp` | `28a980e434c22b4a6b09af551c0b29436a4184e81fc5406a82b69205893a2f33` |
| `include/racecar_driver.h` | `2eeb21d4f0893e8a7508a4d0ee441f80445283769ec972810b1fe31cc8bf7258` |
| `lib/libracecar_driver.a` | `0a93085de45ad939d5ec2f7ca0d841ba9ba2a9dbf834de6cab942f37f5e8212c` |
| `CMakeLists.txt` | `521fb485d6cc5c344b59a64dfa4c764a6cb6675e31e8d568459ada7b1b425646` |
| `package.xml` | `22fb2af9000f06e3a7e752b3fd910c8bb728fa7ef449fdbcd00408aceee8d8fa` |
| `README` | `59c8aef045d1143568f2f7ca4fcfdb362ce463a332aee982b4403e3468798a6c` |

---

## 二、关键版本差异（本地源码 ≠ 车上 0916 记录；车端当前版本待取证）

| 项 | 本地 vendor_src | 车上 0916 记录（历史，待复核） | 证据 |
|---|---|---|---|
| driver 公式偏置 | `×100 + 1500` | `×100 + 1540` | REALCAR_CONTROL_AUDIT §3.3：车上 src 0916 12:23 改 1500→1540 并 `colcon build` |
| install 二进制 | —（本地无编译产物） | `+1540` 版（0916 12:25/26 重编） | 同上 §3.2 时间线 |
| teleop 中位 | 无此文件 | install=1540 / src=1550（未编译） | 同上 §3.3 |

> ⚠️ **本地 vendor_src 是出厂 tar 版（+1500）**；上表"车上"列只是 **0916 的历史记录**。
> **另一会话已在车上改 driver、回退、重编译，车端当前版本未知，须逐文件取证**。
> 本表第一列只代表"出厂源码"，不代表车上运行的真值。这正是要队友回传核对的原因。

---

## 三、待队友回传核对（只读，留空列）

> 「历史假设」列是 0916/0918 的历史记录，**车端可能已被另一会话改动**，只作参照、不作结论。
> 建议命令在车端执行，结果填入「队友实测」列；**任何一项与实际不符就停下回报，不硬猜。**

| # | 核对项 | 历史假设（本地已知） | 队友实测 |
|---|---|---|---|
| 1 | 车上 src 公式偏置 | `~/racecar/src/racecar_driver/src/racecar_driver_node.cpp` 第 45 行 `×100+1540` | |
| 2 | 车上 install 二进制 | `~/racecar/install/racecar_driver/lib/racecar_driver/racecar_driver_node` + `_node_one`（2 个可执行） | |
| 3 | 车上 `libracecar_driver.a` md5 | 本地 8382 B（`sha256sum` 待补） | |
| 4 | 车上运行进程 | `ps -ww -eo pid,args \| grep "[r]acecar_driver"` → 是 node 还是 node_one、启动命令 | |
| 5 | 运行进程订阅话题 | node=订 `/car_cmd_vel`；node_one=订 `/cmd_vel`（决定 nav 链走哪个） | |
| 6 | install 里 driver 公式 | `strings .../racecar_driver_node \| grep -E "^1540$|^1500$"`（粗略判偏置，不可靠则标"未定"） | |
| 7 | 现役启动脚本 | 演示入口 `~/racecar_team/safety_nav_one.sh` 用 node（订 `/car_cmd_vel`，经安全层） | |

---

## 四、为什么这张表重要（一句话）

碰撞根因链（0918）终点是「编码器毛刺→假 odom」，而起点的**速度→PWM 唯一出口在 driver 这一层**；
driver 公式偏置（1500/1540）与订阅话题（node/node_one）任一错位，都会让"限幅 0.10 还是 0.55、
正转还是倒车"的判读失真。先对账 driver，再谈速度链与导航。
