# WSL2 仿真开发环境说明书(2026-09-13 建成)

## 基本信息

| 项 | 值 |
|---|---|
| 发行版名 | `ubuntu2204-ros2` |
| 虚拟磁盘位置 | `D:\install\wsl\ubuntu2204-ros2\`(C 盘零占用) |
| 系统 | Ubuntu 22.04.5 LTS (Jammy) |
| 用户 | `lzx`(密码无,sudo 免密;root 用 `wsl -u root` 进) |
| systemd | 已启用 |
| 资源限制 | 12GB 内存 / 8 核 / 8GB swap(`C:\Users\Administrator\.wslconfig`,改后需 `wsl --shutdown`) |
| 图形 | WSLg(Win11 内置,Gazebo/RViz2 直接弹窗,`DISPLAY=:0`) |

## 已装软件(全部国内源,tools/wsl_setup_ros2.sh 可复现)

ROS2 Humble Desktop / Gazebo Classic 11.10.2 / gazebo-ros-pkgs / Nav2 全家桶 /
slam_toolbox / robot_localization / cv-bridge / colcon / rosdep /
tricycle-steering-controller(阿克曼等效,Humble 无 ackermann 包)。
体检:`python3 tools/check_environment.py` → `"ready": true`(2026-09-13)。

## 工作区

- WSL 内:`~/hwcart_sim`(与 F 盘 `07-仿真系统` 同源,含 .git,remote=GitHub Astrahuaweicar)
- **同步方式**:Windows 侧 ↔ WSL 侧用 `git push/pull`(经 GitHub)或整目录 tar 复制
- ⚠️ **不要在 `/mnt/f/...`(百度同步盘路径)下跑 colcon build**:跨文件系统 IO 慢 10 倍+
  且有文件锁风险;构建一律在 `~/`(ext4)内

## 进入方式

```powershell
wsl -d ubuntu2204-ros2          # PowerShell/Terminal 里进入,默认用户 lzx
wsl --shutdown                  # 全部停掉(改 .wslconfig 后生效用)
```

## 已知处理与坑(环境搭建实录,技术手册素材)

1. `/etc/hosts` 已屏蔽 `fuel.ignitionrobotics.org`(Gazebo 在线模型库,国内连不上
   会拖慢启动报 libcurl 错;本地赛道模型不依赖它)
2. ROS apt 密钥走的三级 fallback(镜像404→jsDelivr要dearmor→keyserver),见脚本注释
3. Windows 代理(127.0.0.1)在 WSL NAT 下不可达:WSL 内要访问 GitHub 时,给 git 配
   代理指向 Windows 主机 IP(`cat /etc/resolv.conf` 的 nameserver + 代理端口),
   或直接从 F 盘复制
4. `Dropped Escape call ulEscapeCode` 日志噪音 = WSLg GPU 直通的正常现象,忽略

## 常用命令速查(WSL 内)

```bash
source /opt/ros/humble/setup.bash     # 已写入 ~/.bashrc,新终端自动生效
ros2 topic list                       # ROS2 图
gazebo                                # 3D 仿真器(GUI)
rviz2                                 # 可视化
cd ~/hwcart_sim/hwcart_sim && colcon build --symlink-install   # 构建工作区
```
