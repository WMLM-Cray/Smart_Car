"""车辆/传感器/赛道参数,单一来源。

标注约定:
  [实测]  2026-09-15/16 在车上测到的
  [代码]  车上 ~/racecar 源码里的值
  [假设]  还没量,先给个合理值 —— 量完改这里
"""
from dataclasses import dataclass


@dataclass
class CarParams:
    # --- 几何 ---
    # [对齐 2026-09-16] 与 Gazebo 主线 params.yaml wheel_base=0.36 一致(SIM_INTEROP 要求
    #   两边同一个值);0.305 是厂家另一版源码的候选值,卷尺实测前不改已验证车辆基线
    wheelbase: float = 0.36
    length: float = 0.56            # [假设] 外形 560mm(规则外形尺寸)
    width: float = 0.35             # [假设] 外形 350mm
    rear_overhang: float = 0.13     # [假设] 后轴到车尾
    max_steer: float = 0.55         # [假设] 前轮最大转角 rad(~31°)
    steer_rate: float = 4.0         # [假设] 舵机转角速率 rad/s

    # --- PWM 接口(与 racecar_teleop.py / web_teleop 一致) ---
    pwm_mid: int = 1550             # [代码] 电调中位
    pwm_fwd_start: int = 1600       # [代码] teleop 注释"can roll";真实起转点未测准
    pwm_rev_start: int = 1360       # [代码] teleop 倒车起步
    pwm_fwd_full: int = 2000        # [代码] teleop 上限
    pwm_rev_full: int = 910         # [代码] teleop 倒车下限(1360-450)
    v_fwd_full: float = 3.0         # [假设] PWM 2000 时车速 m/s —— 待落地测速
    v_rev_full: float = 1.5         # [假设]
    speed_tau: float = 0.35         # [假设] 电机+电调一阶响应时间常数 s
    accel_max: float = 3.0          # [假设] m/s²
    turn_mid: int = 84              # [代码] 舵机中位
    turn_span: int = 60             # [代码] 左右各 60 = 满舵

    # --- 策略输出的速度上限(动作 ±1 对应的 m/s),按课程分阶段收紧/放开 ---
    v_cmd_fwd: float = 1.5
    v_cmd_rev: float = 0.5

    # --- 激光雷达(镭神 M10P,/scan) ---
    lidar_x: float = 0.07           # [代码] base_footprint->laser_link
    lidar_z: float = 0.21           # [假设] 扫描平面离地高度
    lidar_hz: float = 12.0          # [实测] 11.99 Hz
    lidar_points: int = 1667        # [实测] LaserScan ranges 长度,增量 2π/1667
    lidar_min: float = 0.15         # [代码] min_range
    lidar_max: float = 25.0         # [对齐 2026-09-16] 规则硬件量程 25–30m(主线取 25;
                                    #   源码 50 为驱动软件裁剪,记录不改值)
    lidar_sigma: float = 0.01       # [假设] 距离噪声 m

    # --- IMU(/IMU_data)与编码器(/encoder,车尾从动轮) ---
    imu_hz: float = 100.0           # [实测]
    gyro_sigma: float = 0.002       # [假设] rad/s
    accel_sigma: float = 0.05       # [假设] m/s²
    encoder_hz: float = 50.0        # [实测]
    encoder_sigma: float = 0.01     # [假设] m/s


@dataclass
class TrackParams:
    width: float = 1.5              # [假设] 队友示例的赛道宽(锥桶中心距)
    cone_spacing: float = 1.2       # [假设] 队友示例的锥桶间距
    cone_base_radius: float = 0.14  # [假设] 队友用 0.28m 方块代替锥桶
    cone_top_radius: float = 0.025  # [假设]
    cone_height: float = 0.63       # [假设]
    mean_radius: float = 6.0        # 随机环形赛道平均半径 m
    shape_noise: float = 0.25       # 半径扰动幅度(相对)
    min_curvature_radius: float = 1.6  # 中心线最小曲率半径;需 > 半宽 + 车最小转弯半径裕量
