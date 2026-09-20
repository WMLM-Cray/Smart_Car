"""锥桶循迹控制器 —— 移植自 2023 室外 ROS 无人车赛国赛一等奖开源代码。

出处:https://github.com/CodeAlanqian/2023ROS_Autocar
     代码/x1925/vanguardcar/src/lidar_tracking/src/lidar_tracking_nnew.cpp(LGPL)
原版是 ROS1 C++,订阅 /scan 直接发 /car/cmd_vel 的舵机值;这里保持算法不变,
只把输出换成 env 的归一化动作,好和 PPO 在同一套 curriculum.evaluate 下比。

放进来的理由:这是一条在真车上拿过国一、实战调过参的确定性基线。
它和 preview.pure_pursuit 不同 —— 只吃雷达,不读赛道真值,所以能直接上车。

算法(六步,全是几何规则,没有学习):
  1. 找跳变点:ranges[i-1]-ranges[i] >= jump 且 ranges[i] < cone_max_range
  2. 连续性校验:往后 cont_window 个点里要有 cont_need 个和它差 < cont_tol(滤噪)
  3. 极坐标转直角,按横向符号分左右;只留车头前方一个窗口内的
  4. 错分修正:同侧相邻两桶间距 > pair_gap 说明分错了,把远的那段挪到另一侧
  5. 每侧只留最近的 3 个,左右配对(近配近)
  6. 误差 = 三对桶中线偏差的加权和 -> 位置式 PID -> 舵机角

与原版的三处出入(都记在这,别当成 bug):
  - 配对顺序:原版靠 red_p 下标倒序来对齐远近,依赖它那台雷达的 angle_min 约定,
    我没法验证;这里直接按纵向距离排序取近 3 个,是同一个意图的稳妥写法。
  - 补齐逻辑:原版在左右数量不等时复制锥桶凑满 3 对,这里直接只配 min(左,右) 对,
    权重和 rate_sum 一起归一化 —— 等价,少一堆下标操作。
  - 单侧兜底:原版只有一侧有桶时 error=0,也就是直着走。s3 上实测 13/20 的出界
    全是这么来的(失败前 2s 里 36% 的帧配对数为 0),所以改成贴着可见的那一侧走,
    见 _error()。这是唯一一处改了行为的地方。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ConeFollowParams:
    # --- PID:原版 Control.h 的 Init() ---
    kp: float = 0.88
    ki: float = 0.0
    kd: float = 1.2
    inte_max: float = 8.0
    # --- 误差加权:三对桶由近到远 ---
    rate: tuple = (0.8, 2.2, 0.8)
    err_scale: float = 53.33        # 横向误差(m)-> 舵机度数
    # --- 锥桶检测 ---
    jump: float = 2.0               # 距离突变阈值 m
    cone_max_range: float = 2.0     # 超过这个距离的跳变不当锥桶
    cont_window: int = 10           # 连续性校验往后看几个点
    cont_need: int = 6              # 其中要有几个够近
    cont_tol: float = 0.2           # "够近"的阈值 m
    # --- 有效窗口(车体系:x 横向左正,y 纵向前正)---
    x_max: float = 1.5
    y_min: float = -0.2
    y_max: float = 2.0
    pair_gap: float = 1.4           # 同侧相邻桶超过这个距离就认为分错了
    half_width: float = 0.75        # [规则] 赛道宽 1.5m 的一半,单侧兜底时的目标横向距离
    # --- 输出 ---
    turn_span: float = 60.0         # 舵机度数 -> 归一化转角的除数(= CarParams.turn_span)
    throttle: float = 0.5
    corner_angle: float = 20.0      # 舵机角超过这个就减速
    corner_drop: float = 0.06       # 减速比例(原版在 1925 上减 25,即速度增量的 6%)


def detect_cones(ranges: np.ndarray, angles: np.ndarray, p: ConeFollowParams,
                 far: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """一圈测距 -> (左侧锥桶, 右侧锥桶),各 (n,2) 的 [x, y],车体系。

    far 是把 inf/nan 折算成的"远"距离:仿真里没有回波是 inf,原版那台雷达给 0,
    两种都得让 ranges[i-1]-ranges[i] 这个差值算得出来。
    """
    r = np.nan_to_num(np.asarray(ranges, float), nan=far, posinf=far)
    r = np.minimum(r, far)
    n = len(r)
    w = p.cont_window
    # 步骤 1:前一个点和后一个点都比它远 jump 以上,且它本身够近
    i = np.arange(1, n - w - 1)
    jump = ((r[i - 1] - r[i] >= p.jump) & (r[i] < p.cone_max_range)
            & (r[i - 1] - r[i + 1] >= p.jump))
    i = i[jump]
    if not len(i):
        return np.zeros((0, 2)), np.zeros((0, 2))
    # 步骤 2:往后 w 个点里至少 cont_need 个和它差在 cont_tol 内
    nbr = r[i[:, None] + np.arange(1, w + 1)]
    i = i[(np.abs(nbr - r[i][:, None]) < p.cont_tol).sum(1) >= p.cont_need]
    if not len(i):
        return np.zeros((0, 2)), np.zeros((0, 2))
    # 步骤 3:转直角坐标,过窗口
    th = np.asarray(angles, float)[i]
    x, y = r[i] * np.sin(th), r[i] * np.cos(th)
    ok = (np.abs(x) <= p.x_max) & (y > p.y_min) & (y < p.y_max)
    x, y = x[ok], y[ok]
    pts = np.stack([x, y], 1)
    return pts[x > 0], pts[x <= 0]


def _fix_split(near: np.ndarray, far_side: np.ndarray, gap: float):
    """步骤 4:near 里按纵向排序后出现 > gap 的断层,断层之后的都归到另一侧。

    车贴边时同一对桶可能都落在 x 的同一侧,断层就是这种错分的特征。
    """
    if len(near) < 2:
        return near, far_side
    order = np.argsort(near[:, 1])
    near = near[order]
    d = np.hypot(*np.diff(near, axis=0).T)
    cut = np.flatnonzero(d > gap)
    if not len(cut):
        return near, far_side
    m = cut[0] + 1
    return near[:m], np.vstack([far_side, near[m:]]) if len(far_side) else near[m:]


class ConeFollow:
    """policy(obs, env) -> action,签名对齐 curriculum.evaluate。

    obs 不用(它是 48 扇区的压缩版,原版算法要的是原始 1667 点)。
    """

    def __init__(self, params: ConeFollowParams = ConeFollowParams()):
        self.p = params
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def _error(self, left: np.ndarray, right: np.ndarray) -> float:
        """横向误差(舵机度数量纲),正=车偏右、该往左打。

        两侧都有桶:原版的中线偏差 —— 居中时 left.x=+h、right.x=-h,和为 0。
        只有一侧:原版给 0(直着走),s3 上 13/20 的出界就是这么来的。改成贴着看得见的
        那一侧走,离它保持 half_width。整体平移 d 时配对误差是 2d,所以单侧要乘 2 才同量纲。
        """
        p = self.p
        k = min(len(left), len(right))
        if k:
            rate = np.asarray(p.rate[:k])
            return float((left[:k, 0] + right[:k, 0]) @ rate / rate.sum() * p.err_scale)
        side, sign = (left, 1.0) if len(left) else (right, -1.0)
        if not len(side):
            return 0.0                      # 真的一个桶都没有,只能直着走
        m = min(len(side), len(p.rate))
        rate = np.asarray(p.rate[:m])
        dev = side[:m, 0] - sign * p.half_width
        return float(2.0 * (dev @ rate) / rate.sum() * p.err_scale)

    def steer_degrees(self, ranges, angles) -> float:
        """算出舵机相对中位的度数,正=左。上车时直接 turn_mid + 这个值。"""
        p = self.p
        left, right = detect_cones(ranges, angles, p)
        left, right = _fix_split(left, right, p.pair_gap)
        right, left = _fix_split(right, left, p.pair_gap)
        # 步骤 5:每侧留最近的 3 个(按纵向距离)
        left = left[np.argsort(left[:, 1])][:len(p.rate)]
        right = right[np.argsort(right[:, 1])][:len(p.rate)]
        error = self._error(left, right)
        self.integral = min(self.integral + error, p.inte_max)
        out = p.kp * error + p.ki * self.integral + p.kd * (error - self.last_error)
        self.last_error = error
        return out

    def __call__(self, obs, env) -> np.ndarray:
        p = self.p
        angle = self.steer_degrees(env.ranges, env.lidar.angles)
        throttle = p.throttle * (1 - p.corner_drop if abs(angle) >= p.corner_angle else 1.0)
        steer = np.clip(angle / p.turn_span, -1.0, 1.0)
        return np.array([throttle, steer], np.float32)
