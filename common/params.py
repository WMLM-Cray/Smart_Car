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
    # 出厂源码里的 0.305 是别的车,与本车不符。两次实测:队友 2026-09-16 记 0.36,
    # 用户 2026-09-22 复量"更接近 33cm 整"。取 0.33,差异见下。
    #
    # 这个值进自行车模型(vehicle.py)、纯追踪老师(planner.py/preview.py)和
    # /cmd_vel 换算(interop.py),是**策略训练时的被控对象**。candidate_060 是在
    # L=0.305 上训的,改这个值会让它失配:200 局实测 L=0.305 -> 0.956,
    # L=0.36 -> 0.850。策略对轴距很敏感,改完要重新评估,别直接信旧的完赛率。
    #
    # 2026-09-24 用户拍板统一用 0.33,Gazebo 的 racecar_sim/config/params.yaml 同步改了。
    # 2026-09-30 用户按雷达圆心逐项量(车身前端 0.24、后端 0.20,前轮再伸出 0.03、后轮 0.01):
    # 前轴在雷达前 0.205、后轴在雷达后 0.145 -> 轴距 0.35;最前端(前轮)离后轴 0.415,
    # 最后端(后轮)在后轴后 0.065 -> 长 0.48。车尾里程计展开再伸出 0.17,是中间一根细杆,不进车体矩形。
    wheelbase: float = 0.35         # [实测 2026-09-30]
    length: float = 0.48            # [实测 2026-09-30] 后轮 -> 前轮外沿
    width: float = 0.35             # [实测 2026-09-30] 车身宽 350mm
    rear_overhang: float = 0.065    # [实测 2026-09-30] 后轴到车尾(后轮外沿)
    # 舵机值 -> 等效自行车前轮角(度,左正),2026-09-24 手推实测,
    # docs/实测/2026-09-24-steering/转向标定.md。不是线性的:近满舵 15 个刻度
    # 只多 2~3°。点间线性插值。零点 2026-10-12 走廊直线实测改到 80(见 turn_mid),
    # 81 插值出 +0.3°,和当初手推测到的 81 = +0.3° 一致。
    steer_curve: tuple = ((21, -21.7), (36, -18.6), (51, -13.4), (66, -5.8), (76, -1.2),
                          (80, 0.0), (86, 1.9), (96, 5.6), (111, 11.9), (126, 17.9),
                          (141, 20.5))
    # 以前是 [假设] 0.55(~31°),实测右满舵 21.7°。现在只当物理钳位用,
    # 动作 -> 转角走 steer_curve(pwm.action_to_steer)。
    max_steer: float = 0.379        # [实测 2026-09-24] = 21.7°
    steer_rate: float = 4.0         # [假设] 舵机转角速率 rad/s

    # --- PWM 接口(与 racecar_teleop.py / web_teleop 一致) ---
    # [实测 2026-09-22 晚] 原来写 1550,抄的是厂家 teleop 脚本里的"中位" ——
    # 那个值本身就带着轻油门。三条证据:
    #   1. 厂家驱动 racecar_driver_node.cpp 自己的换算是 linear.x*100 + 1540,
    #      速度 0 对应 **1540**;
    #   2. 落地 ramp_curve:满电时降到 1550 车仍以 561 mm/s 往前爬,一直到
    #      1544 才滑停;
    #   3. coast_test:1545 / 1530 / 1510 表现完全一致,都是不出力。
    # 后果不只是"停不住":pwm_mid 是速度映射的零点,偏 10 个 PWM 等于每条指令
    # 都带一个常量油门,而且所有脚本的 neutral_exit() 发的"中位"根本不停车。
    pwm_mid: int = 1540             # [实测 2026-09-22]
    # 电调**不主动刹车**。原注释说"低于 1600 会刹停,不是滑行",2026-09-22 晚
    # coast_test 实测推翻:从 0.59 m/s 松到 1545/1530/1510,减速度都是
    # 0.68~0.78 m/s²(纯滚阻),滑 0.25m、0.8s 停下,三个值毫无区别。
    # 也就是说减速只能靠滑行 —— vehicle.py 用 accel_max 双向限幅**偏乐观**,
    # 制动侧的真值是 0.73 m/s²,不是 accel_max 的 3.0。
    # [实测] 0.59 m/s 松油 0.73(2026-09-22);0.91 m/s 松油 0.78、滑 0.56m(2026-09-23,11.24V)。
    # 基本不随车速变(像库仑摩擦),取小的那个:拿它算防撞滑停距离时偏保守。
    coast_decel: float = 0.73       # [实测] 松油自由滑行减速度 m/s²
    # 刹车(PWM 低于中位区间下沿 ~1495;docs/实测/2026-10-12-brake):1470/1440/1400 一样,1.5 m/s 起 ~1.3、2 m/s 起 ~1.5、
    # 1 m/s 起 ~2.6 m/s²(已经混进倒车)。刹到低速还给低 PWM 就变倒车(倒退 0.1~0.2 m),所以只在这个车速以上刹,低了回中位滑行。
    brake_release_speed: float = 0.5   # [实测推的规则] m/s,车上节点和仿真 SpeedPlant 共用
    brake_pwm: int = 1470           # [实测] 车上执行 BRAKE_CMD 发的 PWM:1470/1440/1400 刹车力一样,取离中位最近的
    # 截停距离随车速放大:0.25 m 只在 0.7 m/s 下实车验证过;更快时加上多出来的"延迟 + 刹车"距离,截停时真刹车。
    guard_base_speed: float = 0.7   # [实测] 截停 0.25 m 验证过的车速
    guard_latency: float = 0.2      # [实测] 指令到车动 ~0.2 s(见 vehicle/road_car_node.py LAUNCH_FROM 注释)
    guard_brake_decel: float = 1.3  # [实测] 1.5 m/s 起刹 ~1.3 m/s²,取小的(docs/实测/2026-10-12-brake)
    policy_mode_caps: tuple = (0.7, 1.2, 2.5)   # 网页 Sloth / Roaming / MAD MAX 对应的策略限速 m/s
    v_fwd_full: float = 3.0         # [假设] 基础 Car 模型速度钳位；road_v2 实际速度由 SpeedPlant 决定
    v_rev_full: float = 1.5         # [假设]
    speed_tau: float = 0.35         # [假设] 电机+电调一阶响应时间常数 s
    accel_max: float = 3.0          # [假设] m/s²
    # 横向加速度上限 = 摩擦系数 x g。超过这个值轮胎抓不住,车推头(实际转弯半径大于
    # 指令值)。运动学模型本身没有这个约束 —— 不加的话 5m/s 也能过 1m 半径的弯,
    # 那是 2.5g,物理上不可能,训出来的策略上车就飞。
    # 0.8g:2026-09-19 按用户判断取的值(室内光滑地面小型 RC 车常见 0.8~1.2)。[假设]
    a_lat_max: float = 2.6          # [实测 2026-10-24] 横向抓地:实际 = a·tanh(要求/a),a≈2.6(runs/drive_1024c,原假设 7.85)
    # 转向使用实测非线性曲线，不能按固定角度/刻度估算偏置。
    # 换过舵机/拉杆/舵盘齿位就要重测。
    # 原来是 84(从厂家源码抄的,没在这台车上对过)。83 偏左明显,80~82 之间
    # straight_trim 分辨不出来 —— 每次只跑 1.9m,1°/m 总共才 2°,和陀螺积分
    # 3 秒的噪声同量级(82 两次是 +0.6/-0.6)。80 是用户复跑后体感选的
    # (同一天先选过 81,复跑改口 80;两者在数据上本来就分不开)。
    # 想钉死就 straight_trim.py --turns 79 80 81 --sec 6 --repeat 5,跑长一点。
    # 2026-09-23 落地测曲线时 80 两次走偏(一次左一次右,多半是摆车角度),
    # 用户判断整体偏右,改 81；角度须按非线性实测表换算。
    # 2026-10-12 走廊直线实测:81 偏左(半径 ~65 m)、80 偏右(~100 m),真中位 ~80.4,改 80。
    turn_mid: int = 80              # [实测 2026-10-12]
    turn_span: int = 60             # [代码] 左右各 60 = 满舵

    # --- 策略输出的速度上限(动作 ±1 对应的 m/s),按课程分阶段收紧/放开 ---

    # --- 激光雷达(镭神 M10P,/scan) ---
    lidar_x: float = 0.145          # [实测 2026-09-30] 后轴 -> 雷达(厂家 TF 写的 0.07 不对)
    lidar_z: float = 0.28           # [实测 2026-09-30] 扫描平面离地高度
    lidar_hz: float = 12.0          # [实测] 11.99 Hz
    lidar_points: int = 1667        # [实测] LaserScan ranges 长度,增量 2π/1667
    lidar_min: float = 0.15         # [代码] min_range
    lidar_max: float = 50.0         # [代码] 驱动软件裁剪
    lidar_sigma: float = 0.01       # [假设] 距离噪声 m

    # --- 摄像头(BEV 用;位置以后轴为原点,和 lidar_x 同一坐标系) ---
    # 2026-10-09/10 实车标定,方法和误差见 docs/实测/相机标定/相机标定.md,机器可读版 calibration/camera_640x480.json。
    # 换停放位置俯仰会变 ~1°(悬架):地面投影误差 ≈ 距离²/高度 × 角度,1° 在 1 m 处 ~7 cm、3 m 处 ~0.7 m。
    camera_x: float = 0.335         # [实测] 后轴 -> 光心(镜头最前端在雷达中心前 0.20,光心再往后约 1 cm)
    camera_z: float = 0.24          # [实测] 光心离地高度
    camera_hfov: float = 1.2270      # [队友 2026-10-08] 水平视场 70.3°(标定 fx 453 -> 70°,一致)
    camera_vfov: float = 0.9739      # [队友 2026-10-08] 垂直视场 55.8°
    camera_pitch: float = -0.0297   # [实测] 光轴下俯角 rad:负 = 朝上 1.70°
    camera_yaw: float = 0.0361      # [实测] 光轴偏航 rad,正 = 朝左 2.07°
    camera_roll: float = -0.0386    # [实测] 横滚 rad(-2.21°)
    camera_size: tuple = (640, 480)                      # [实测] 实际取流分辨率
    camera_K: tuple = (453.1, 451.0, 318.3, 232.8)       # [实测] fx fy cx cy(640×480)
    camera_dist: tuple = (-0.3769, 0.1574, 0.0016, 0.0006, -0.0331)   # [实测] k1 k2 p1 p2 k3,桶形
    camera_hz: float = 30.0         # [假设] 帧率(Gazebo 同值)
    yolo_range: float = 5.0         # [假设] 检测有效距离 m:再远框太小、落地投影误差太大

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
    # 2026-10-01 用户:比赛用最常见的 70cm 雪糕桶。底座方板很薄、在雷达平面(0.28)以下,不建模;
    # 锥身底/顶直径按这类桶常见规格 ~28cm / ~5cm 取,雷达高度处截面直径 ~19cm。
    cone_base_radius: float = 0.14  # [常见规格] 锥身底部直径 ~28cm
    cone_top_radius: float = 0.025  # [常见规格] 顶口直径 ~5cm
    cone_height: float = 0.70       # [用户 2026-10-01] 70cm 雪糕桶
    mean_radius: float = 6.0        # 随机环形赛道平均半径 m
    shape_noise: float = 0.25       # 半径扰动幅度(相对)
    min_curvature_radius: float = 1.6  # 中心线最小曲率半径;需 > 半宽 + 车最小转弯半径裕量


@dataclass
class RuleTrackParams(TrackParams):
    """按比赛规则的元素拼赛道(track_rules.py)。[规则] 项见该文件头部出处。"""
    turn_radius: float = 2.0            # [规则] 内径 1.25 / 外径 2.75 -> 中心线 2.0
    turn_angles: tuple = (45.0, 90.0, 135.0)   # [规则] 只有这三种转角
    # --- 布局:一条大直道 + 两端 180° 大弯 + 回程侧连续 S 弯(规则图 1 的长条形)---
    main_straight_min: float = 6.0      # [规则图1] 主直道明显比其它直段长(由闭合定出来,超范围就重抽)
    main_straight_max: float = 12.5     # [规则] 场地 16.8m,留出两端掉头的余量
    bend_radius_min: float = 1.0        # [图1] S 弯比圆弯紧得多;下限对车(实测转角对应半径约0.83–0.88m)仍有裕量
    bend_radius_max: float = 1.8        # [图1]
    cross_min: float = 1.5              # [假设] 方角中间那段横直道至少这么长,否则看不出是直角
    height_min: float = 3.5             # [图1] 主直道到回程直道的间距(赛道"厚度")
    height_max: float = 6.0             # [图1]
    s_bends_min: int = 2                # [规则图1] 回程侧的台阶组数(每组 2 个镜像对 = 4 个弯)
    s_bends_max: int = 2                # 连两端的弯,一共 10-12 个
    square_end_chance: float = 0.7      # [图1] 端部做成方角(两个直角)的概率,其余是大圆弯
    end_radius: tuple | None = None     # 方角端弯半径范围;None = 和 S 弯同一个范围(road.py L5 用它出尖方角)
    right_angle_ratio: float = 0.7      # [规则] 折角里直角的比例,赛道整体偏方正
    jog_min: float = 1.2                # [假设] 矩形台阶横向段长度
    jog_max: float = 3.0                # [假设]
    straight_min: float = 0.4           # [假设] 弯与弯之间的短直段下限
    straight_max: float = 1.2           # [假设] 回程侧两弯之间留得短,整圈才塞得进 16.8m
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
