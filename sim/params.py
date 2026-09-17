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
    wheelbase: float = 0.305        # [假设] 队友从厂家另一版代码抄的 L,需卷尺复核
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
    lidar_max: float = 50.0         # [代码] 驱动软件裁剪
    lidar_sigma: float = 0.01       # [假设] 距离噪声 m

    # --- IMU(/IMU_data)与编码器(/encoder,车尾从动轮) ---
    imu_hz: float = 100.0           # [实测]
    gyro_sigma: float = 0.002       # [假设] rad/s
    accel_sigma: float = 0.05       # [假设] m/s²
    encoder_hz: float = 50.0        # [实测]
    encoder_sigma: float = 0.01     # [假设] m/s


@dataclass
class TrackParams:
    width: float = 1.5              # [规则] 赛道宽 1.5m(规则第2-6页;圆形赛道内径1.25/外径2.75 也是这个宽)
    cone_spacing: float = 1.15      # [图1量得] 锥桶间距 ≈ 赛道宽/1.32,约 1.1-1.15m
    cone_base_radius: float = 0.14  # [假设] 队友用 0.28m 方块代替锥桶
    cone_top_radius: float = 0.025  # [假设]
    cone_height: float = 0.63       # [假设]
    mean_radius: float = 6.0        # 随机环形赛道平均半径 m
    shape_noise: float = 0.25       # 半径扰动幅度(相对)
    min_curvature_radius: float = 1.6  # 中心线最小曲率半径;需 > 半宽 + 车最小转弯半径裕量


@dataclass
class RuleTrackParams(TrackParams):
    """按比赛规则的元素拼赛道(track_rules.py)。[规则] 项见该文件头部出处。"""
    turn_radius: float = 2.0            # [规则] 内径 1.25 / 外径 2.75 -> 中心线 2.0
    turn_angles: tuple = (45.0, 90.0, 135.0)   # [规则] 只有这三种转角
    # --- 布局:一条大直道 + 两端 180° 大弯 + 回程侧连续 S 弯(规则图 1 的长条形)---
    main_straight_min: float = 5.5      # [规则图1] 主直道明显比其它直段长
    main_straight_max: float = 8.5      # [假设]
    bend_radius_min: float = 1.0        # [图1] S 弯比圆弯紧得多;下限对车(最小转弯半径0.53m)仍有裕量
    bend_radius_max: float = 1.8        # [图1]
    end_radius_min: float = 1.6         # [假设] 端部掉头大弯,图上比普通弯大
    end_radius_max: float = 2.6         # [假设]
    s_bends_min: int = 2                # [规则图1] 回程侧是连续小 S 弯(每组 2 个镜像对 = 4 个弯)
    s_bends_max: int = 4                # [规则图1] 图上回程侧弯挺多,连两端掉头一共 10-14 个弯
    square_end_chance: float = 0.7      # [图1] 端部做成方角(两个直角)的概率,其余是大圆弯
    right_angle_ratio: float = 0.7      # [规则] 折角里直角的比例,赛道整体偏方正
    jog_min: float = 1.2                # [假设] 矩形台阶横向段长度
    jog_max: float = 3.0                # [假设]
    straight_min: float = 0.4           # [假设] 弯与弯之间的短直段下限
    straight_max: float = 4.0           # [规则] 图 9 有 4m 直段标注
    max_extent: float = 16.8            # [规则] 图 9 总横向 16.8m
    max_short_side: float = 8.0         # [图1] 赛道是长条形,短边明显小于长边
    # 队友读规则图 9 记的"局部 1.25m"存疑:1.25 也正好是圆形赛道的内径,可能是看串了。
    # 拿到规则原文前默认不收窄(narrow_fraction=0),要开就把它调成 0.08。
    narrow_width: float = 1.25          # [存疑] 见上
    narrow_start: float = 0.35          # [假设] 收窄段起点(占一圈的比例)
    narrow_fraction: float = 0.0        # [存疑] 收窄段长度占比,默认关
    n_boards: int = 5                   # [规则] 5 块随机蓝挡板
    board_length: float = 0.8           # [假设] 挡板长度 m
    board_height: float = 0.5           # [假设] 挡板高度 m
