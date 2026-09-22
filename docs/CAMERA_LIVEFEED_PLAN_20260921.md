# 相机实时画面 —— 只读推流方案（2026-09-21，优先级第一）

> 目标：复用车上**已存在的图像话题**，用 `web_video_server` 做**只读** HTTP 推流，队友浏览器观看。
> **不启动导航 / 运动链**。厂家 `camer.launch.py` 只是候选入口，不预设 USB / `/dev/video0` / `/image_raw`；
> 此前 CSI/MIPI 线索须现场排除后再定。本机不连车，事实由队友在车上只读采集后回传。

## 1. 队友只读核对（回传这 4 点，缺一个不动手）

车机执行（全部只读，不起运动/导航）：

```bash
# ① 设备与硬件接口（判 USB 还是 CSI/MIPI）
ls -l /dev/video* 2>/dev/null
v4l2-ctl --list-devices 2>/dev/null
dmesg 2>/dev/null | grep -iE 'uvc|csi|mipi|camera|video' | tail -20

# ② 现有采集进程（有没有已在跑的）
ps aux | grep -iE 'usb_cam|camera|gst|libcamera|rpicam|v4l|csi|mjpg|ros2' | grep -v grep

# ③ 图像话题 + 类型/编码/分辨率/是否出帧
ros2 topic list | grep -iE 'image|camera|compressed'
ros2 topic info /image_raw -v        # 示例；对每个候补话题都跑 -v
ros2 topic hz   /image_raw --window 20

# ④ 已装相关包
dpkg -l 2>/dev/null | grep -iE 'usb-cam|usb_cam|web-video|image-transport|libcamera|v4l'
```

回传 4 点：
1. **相机硬件接口**：USB 还是 CSI/MIPI？设备节点是哪个 `/dev/video?`。
2. **现成采集进程**：有没有已在跑的（节点名/包）；没有写"无"。
3. **图像话题**：实际话题名 + 类型（`Image`/`CompressedImage`）+ `encoding` + 分辨率 + 是否在出帧。
4. **已装包**：`usb_cam` / `web_video_server` / `image_transport` 谁在谁不在。

## 2. 分支 A：已有图像话题（默认目标）

`web_video_server` 只**订阅**图像话题、转发/重编码成 HTTP，不碰控制链、不起导航。

- 安装：`sudo apt install -y ros-humble-web-video-server`
- 核对安装：`ros2 pkg list | grep web_video_server`
- 核对端口占用：`ss -lntp | grep :8080`（默认 8080，被占则换 `-p port:=8090`）
- 启动：`ros2 run web_video_server web_video_server`（默认 0.0.0.0:8080）
- 访问：队友浏览器 `http://<车机IP>:8080/`（话题列表），或直接 `http://<车机IP>:8080/stream?topic=<实际图像话题>`
- 停止：`Ctrl+C`（或 `pkill -f web_video_server`）
- **话题兼容**：只接收 `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage`。
- **延迟/负载限制**：raw Image 需服务端软件重编码成 MJPEG；320×240 负载小，分辨率越高 CPU 越高；
  若存在 `CompressedImage`/`/compressed` 话题应**优先用它**（直接转发，负载和延迟都更低）。

## 3. 分支 B：没有图像话题

- **不**直接 `apt install usb-cam` 碰运气。
- 按 §1 事实定硬件：**USB** → 候选 `ros2 launch racecar camer.launch.py`（即 usb_cam，设备号以 `ls -l /dev/video*` 为准）；
  **CSI/MIPI** → 本仓库只含 usb_cam，无 CSI 采集程序，须队友回传硬件型号，我再对照准备匹配采集，不回传不预设。

## 4. 最终交付格式（收到事实后一次给全）

1. 实际图像源（设备/节点/话题）
2. 启动方式（采集 + web_video_server 各一条）
3. 浏览器地址
4. 停止方法
5. 延迟/负载限制