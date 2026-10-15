"""路段生成器:把直道、弯、掉头、台阶、变宽、无边界段串成一条不闭合的路。

走廊和赛道是同一套东西,只是边界材质不同(墙 / 锥桶 / 没有)。赛道 = 同样的路段
拼成闭环再换成锥桶,放在课程最后。

几何约定:
  中心线按 DS 等距采样,左右边界 = 中心线 ± 法向 · 半宽。
  弯的中心线半径 = 内侧半径 r_in + 半宽,所以 r_in=0 就是走廊的尖内角
  (外侧是半径为路宽的圆角,比真实方角空间小,偏保守)。
  车走不走得过去不在这里判断,见 road_check.py。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

DS = 0.05                   # 中心线采样间距 m
NONE, WALL, CONE, EDGE = 0, 1, 2, 3  # 边界材质;EDGE = 路面换材质(地板/草地、马路牙子):不挡雷达、没有锥桶,只有地面分割看得出
WALL_HEIGHT = 1.0           # 墙比雷达扫描面(0.21m)高就行
WALL_STEP = 0.2             # 墙按这个间距切成线段给射线求交


@dataclass(frozen=True)
class Level:
    """一级课程 = 一组生成参数的范围。"""
    name: str
    width: tuple = (1.6, 2.5)           # 路宽 m
    angles: tuple = (45.0, 90.0)        # 弯角可选值
    r_in: tuple = (1.0, 2.0)            # 弯内侧半径 m
    sharp_chance: float = 0.0           # 弯是尖角(r_in 0~0.1)的概率
    uturn_chance: float = 0.0           # 一个元素是 180° 掉头的概率
    uturn_r_in: tuple = (0.5, 1.25)
    jog_chance: float = 0.0             # 台阶(两个反向 90°)的概率
    taper_chance: float = 0.0           # 变宽段的概率
    gap_chance: float = 0.0             # 直道做成无边界段的概率
    gap_len: tuple = (3.0, 6.0)
    one_side_none_chance: float = 0.0   # 某一侧没有边界的概率
    n_elems: tuple = (2, 3)             # 起点直道之后的元素个数
    straight: tuple = (1.5, 4.0)        # 元素之间的直道长
    cone_spacing: tuple = (0.5, 0.9)
    wall_chance: float = 0.5            # 一侧用墙(否则锥桶)的概率
    edge: bool = False                  # 两侧边界全用 EDGE(无锥桶无墙的路,只能靠地面分割认路沿)
    # --- 闭环赛道(L5):按规则元素拼的闭环(track_rules.make_rule_track),折在 16.8m 场地里 ---
    loop: bool = False
    loop_width: tuple = (1.4, 1.6)      # [规则] 1.5m,上下留一点
    loop_bend: tuple = (1.0, 1.4)       # 弯中心线半径下限 = 它 -0.25、不小于半宽(=内侧尖角,赛道 A 左端方角)
    # 锥桶间距分三种位置抽:赛方参考图(用户 2026-09-25 量)直道约 0.61、弯内侧最密约 0.45;
    # 同一张图按 16.8m 比例尺量直道是 ~1.13(和 params.py 的 1.15 一致)。两种说法都覆盖到。
    # 上限 1.5:规则只要求桶距 < 道宽 1.5m;官方 A 实测 0.48~1.475(原来训练最大 1.2,
    # A 左上方角 1.3~1.47 的稀桶没见过,四个模型全从缝里冲出去,2026-09-25)
    straight_spacing: tuple = (0.55, 1.5)
    inner_spacing: tuple = (0.4, 0.65)
    outer_spacing: tuple = (0.6, 1.5)
    sharp_end_chance: float = 0.6       # 方角端用尖角(半径 = 半宽 ~ 半宽+0.25)的概率
    # --- 开口方角(L6):尖 90° 的外角做成方角,外侧桶稀(1.1~1.5m),正前方留一个缺口 ---
    # 赛道 A 左上角:外角两桶 (0,5.5)/(0,4.1) 隔 1.4m,雷达看正前方像一条继续往前的路,
    # 所有模型都直冲出去;和实车走廊尽头卡死是同一种情形(2026-09-25)。
    gap_corners: bool = False
    gap_spacing: tuple = (0.4, 1.5)     # 方角外侧桶距
    # 方角缺口后面摆一段"假走廊"(来车方向延长线上两排桶)的概率。赛道 A 左上角在缺口后面
    # 10m 内加一排桶,v2_g 通过率 0.83 -> 0.03:学生看缺口后面像不像路来决定拐不拐,
    # 而训练里缺口后面几乎总是空的(2026-09-26)。实车场地上缺口后面是别的赛段、人、杂物
    decoy_chance: float = 0.0
    # 方角前那段直道(缓 S 弯后的 / 纯直道)长度;None = 原来的 1.0~3.0 / 1.5~4.0
    gap_lead: tuple | None = None
    # --- 闭环形状旋钮(只有 loop=True 用)。None = 沿用 RuleTrackParams 默认值,历史行为逐位不变。
    # 2026-09-29:此前 generate_loop 只暴露 width/bend/桶距,其余形状参数全锁在默认值上,
    # 于是不管抽多少种子,闭环永远是同一个"长条形矩形回路"家族(24 条叠图实测)。
    # 这几个旋钮把形状空间打开,让"直角区 / 大圆弯掉头 / 台阶组数 / 长短边比例"真的随机。
    loop_height: tuple | None = None      # 主直道到回程直道的间距(赛道"厚度")
    loop_square_end: tuple | None = None  # 端部做成方角(两个直角)的概率范围
    loop_right_angle: tuple | None = None # 回程折角里直角的比例范围,其余抽 45°/135°
    loop_s_bends: tuple | None = None     # 回程台阶组数范围(每组 2 个镜像对 = 4 个弯)
    loop_main: tuple | None = None        # 主直道长度范围(闭合解出来,超范围重抽)
    loop_bend_max: float | None = None    # S 弯半径上限(默认 1.8)
    loop_turn_angles: tuple | None = None # 折角可选角度(默认 45/90/135)
    # --- 路中障碍(2026-10-15):路面上摆雷达扫得到的箱子/板子,要从旁边绕过去 ---
    # 实车走廊:展板、叠起来的垫子、椅子迎面摆着,模型照直开到截停 —— 训练里唯一"迎面是墙"的是直角弯,
    # 路一定往左或右拐;路中间突然立着东西、绕过去再回到路上的情形从没见过。
    obstacles: tuple = (0, 0)              # 每条路的障碍个数范围
    obstacle_w: tuple = (0.2, 0.7)         # 横向宽 m
    obstacle_l: tuple = (0.05, 0.5)        # 纵向深 m(0.05 = 一块薄板)
    # --- 直角口袋(2026-10-15):尖 90° 弯出去的那条走廊往反方向多伸出一截死胡同(T 字口一边短一边长)---
    # 实车:从门洞出来进横走廊,一头几米外就封死、另一头是路。训练里的路全是单线,拐弯外侧直接是墙角,没见过。
    # 深度上限 2.5:再深末端墙就出了 BEV(左右 4 m),两边看起来一样,老师往一边拐的标签学不来
    pocket_chance: float = 0.0             # 有墙的尖 90° 弯做成口袋的概率
    pocket_depth: tuple = (0.3, 2.5)       # 口袋从来车路外侧墙算起的深度 m


LEVELS = (
    Level('L1'),
    Level('L2', width=(1.2, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.3, 2.0),
          uturn_chance=0.15, jog_chance=0.15, taper_chance=0.1, n_elems=(3, 5),
          straight=(0.8, 4.0)),
    Level('L3', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(3, 6), straight=(0.4, 3.0)),
    Level('L4', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          gap_chance=0.2, one_side_none_chance=0.1, n_elems=(4, 7), straight=(0.3, 3.0),
          cone_spacing=(0.4, 1.2)),
    Level('L5', loop=True),
    Level('L6', gap_corners=True, width=(1.3, 1.7), wall_chance=0.2, decoy_chance=0.5),
    # 只做验证:稀桶方角(赛道 A 左上角 1.3~1.47)。L6 整体 0.98 的两个存档在 A 上一个 100% 一个 0%,
    # 差别全在这种角上,L6 里它占比小、看不出来(2026-09-26)
    Level('L6h', gap_corners=True, width=(1.3, 1.7), wall_chance=0.0, gap_spacing=(1.2, 1.5),
          decoy_chance=0.5),
    # 只做评估(放最后,不动前面各级在断点里的下标):L4 去掉胶带段(无边界缺口、单侧无边界)。
    # 用户 2026-09-25:先不测胶带
    Level('L4c', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(4, 7), straight=(0.3, 3.0), cone_spacing=(0.4, 1.2)),
    # 稀桶闭环:L5 里外侧/直道桶距 1.2~1.5 的那三分之一通过率 0.91~0.93,其余 0.97~0.99(v2_i,2026-09-26)
    Level('L5s', loop=True, straight_spacing=(1.1, 1.5), outer_spacing=(1.1, 1.5)),
    # 高速冲向方角:方角前直道加长到 4~7m,车能加到 1.5 m/s 以上再进角。road_v2.5 第 2 档
    # (限速 2.2)在赛道 A 左上角限速 1.8 时 16 局撞 6~9 局:老师离角 2m 已减到 0.6、打满舵,
    # 学生还在加速到 1.66、方向几乎不动 —— 训练里方角前直道短,高速进方角的状态几乎没见过(2026-09-29)
    Level('L6f', gap_corners=True, width=(1.15, 1.5), wall_chance=0.0, gap_spacing=(1.2, 1.5),
          decoy_chance=0.5, gap_lead=(4.0, 7.0)),
    # --- 以下为本次新增,一律追加在末尾,不动已有各级的下标(断点里的 level 是下标)---
    # spica 主训练级:闭环规则赛道,弯更紧。bend 下限 = max(半宽, bend-0.25) ≈ 0.9,
    # 即中心线曲率地板被压到 v1 训练时的 1.0 以下(实测 92% 的赛道 R_min < 1.0,
    # 28% < 车的转弯极限 0.83),而路宽不动(窄是另一个失效轴,不混进来)。
    # 老师 25/25 出圈,标签存在。2026-09-28
    Level('L5c', loop=True, loop_bend=(0.9, 1.2)),
    # --- 闭环形状家族(2026-09-29)---
    # 起因:闭环赛道的形状空间没打开 —— generate_loop 只暴露
    # width/bend/桶距,其余形状参数全锁在 RuleTrackParams 默认值上,24 条叠图是同一个长条形家族。
    # 旋钮在 Level 的 loop_* 字段,None = 老行为逐位不变。下面是几个"有价值"的家族:
    # 硬标准 = 老师能过(标签存在)、形状确实不同、R_min 不塌到车极限 0.83m 以下。
    # 直角为主:端部方角 + 全直角台阶 + 只抽 90°,高度拉大让直角区看得出来
    Level('L5ra', loop=True, loop_bend=(0.9, 1.2), loop_square_end=(1.0, 1.0),
          loop_right_angle=(1.0, 1.0), loop_turn_angles=(90.0,),
          loop_height=(5.0, 7.0), loop_s_bends=(2, 3)),
    # 大圆弯掉头:端部不用方角,折角只抽 45/135,柔和,和 L5ra 是两个极端
    Level('L5rd', loop=True, loop_bend=(0.9, 1.2), loop_square_end=(0.0, 0.0),
          loop_right_angle=(0.0, 0.0), loop_turn_angles=(45.0, 135.0),
          loop_height=(4.0, 5.5), loop_s_bends=(2, 2)),
    # 台阶密集:回程侧 4~5 组连续 S 弯(规则图 1 的样子),主直道拉长才塞得下
    Level('L5ms', loop=True, loop_bend=(0.9, 1.2), loop_s_bends=(4, 5),
          loop_height=(6.0, 8.0), loop_main=(9.0, 14.0)),
    # 接近方形:短主直道 + 大高度,和长条形是两种家族
    Level('L5sq', loop=True, loop_bend=(0.9, 1.2), loop_height=(7.0, 9.0),
          loop_main=(7.0, 10.0), loop_s_bends=(2, 2)),
    # C 的失败点:(0.5,4.2) 密桶方角(b062-b063 0.50m),学生按"方角=稀桶"直接撞上去。
    # 与 L6f 相反:外侧桶距压到 0.45~0.65(生成器默认 0.4~1.5 只覆盖到 1.5,密端太稀)。
    Level('L6d', gap_corners=True, width=(1.2, 1.6), wall_chance=0.0, gap_spacing=(0.45, 0.65),
          decoy_chance=0.5, gap_lead=(2.0, 5.0)),
    # 无锥桶路(bev_v2 对照,2026-10-10):几何同 L3 / L4c,两侧全是 EDGE —— 雷达什么都扫不到,
    # 路沿只在地面分割层里。验证"没摆赛道时靠地面分割沿路面开"。
    Level('LE3', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(3, 6), straight=(0.4, 3.0), edge=True),
    Level('LE4', width=(0.9, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(4, 7), straight=(0.3, 3.0), edge=True),
    # 路中障碍(2026-10-15):L3 / LE3 的几何 + 每条路 1~3 个箱子/板子。路宽下限放到 1.3,窄路上再摆障碍就过不去了
    Level('LO3', width=(1.3, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(3, 6), straight=(0.8, 3.5), obstacles=(1, 3)),
    Level('LOE3', width=(1.3, 2.5), angles=(45.0, 90.0, 135.0), r_in=(0.0, 2.0),
          sharp_chance=0.4, uturn_chance=0.15, jog_chance=0.2, taper_chance=0.15,
          n_elems=(3, 6), straight=(0.8, 3.5), edge=True, obstacles=(1, 3)),
    # 直角口袋(2026-10-15):全是墙的直角走廊,70% 的尖直角弯外侧多一截死胡同。按深度分三档从浅到深依次训。
    # 只做有墙的:没墙时相机(3 m、视场有限)看不到口袋末端,深口袋和真岔路分不出来
    *[Level(name, width=(0.9, 2.0), angles=(90.0,), sharp_chance=1.0, jog_chance=0.3, n_elems=(3, 5),
            straight=(1.0, 3.0), wall_chance=1.0, pocket_chance=0.7, pocket_depth=d)
      for name, d in (('LT3a', (0.2, 0.7)), ('LT3b', (0.7, 1.4)), ('LT3c', (1.4, 2.5)))],
)


@dataclass
class Road:
    centre: np.ndarray          # (n,2)
    heading: np.ndarray         # (n,)
    s: np.ndarray               # (n,) 弧长
    half: np.ndarray            # (n,) 半宽
    mat_left: np.ndarray        # (n,) 材质
    mat_right: np.ndarray
    cone_radius: float = 0.14
    walls: np.ndarray = field(default_factory=lambda: np.zeros((0, 2, 2)))
    cones: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    elems: list = field(default_factory=list)     # 生成记录,便于复现和统计
    # 官方赛道:路面 = 外边界多边形内、内边界多边形外。给了就用它判出界,不用半宽 ——
    # 方角处车走进角落是合法的,但离平滑后的中心线超过半宽(2026-09-25 赛道 A 全判成出界)
    region: tuple | None = None
    # 左右各自的半宽。官方赛道的走廊并不以中心线对称:A 在左下直角弯左侧只剩 0.48m、
    # 右侧有 1.65m(总宽 2.13m),旧的 half = min(左,右) 把右边 1.16m 白白扔掉,规划可行域
    # 只剩 0.019m —— A 上全灭的直接原因(2026-09-29)。给 None 就退回对称的 half。
    half_left: np.ndarray | None = None
    half_right: np.ndarray | None = None
    # 路中障碍:每个 4 个角 (n, 4, 2)。墙段已并进 walls(雷达、碰撞);规划用的可行域在障碍那一截从被挡的一侧收窄
    # (half_left/half_right + 材质记成 WALL),假地面分割扣掉它和它挡住的那一扇
    obstacles: np.ndarray = field(default_factory=lambda: np.zeros((0, 4, 2)))
    # 直角口袋:每个一个多边形 (n, 5, 2)(拐角外侧 + 死胡同),墙已并进 walls;假地面分割把它算地面。
    # 规划和出界判断不认它(开进口袋 = 走错)
    pockets: np.ndarray = field(default_factory=lambda: np.zeros((0, 5, 2)))

    @property
    def normal(self) -> np.ndarray:
        return np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)

    @property
    def half_l(self) -> np.ndarray:
        return self.half if self.half_left is None else self.half_left

    @property
    def half_r(self) -> np.ndarray:
        return self.half if self.half_right is None else self.half_right

    @property
    def left(self) -> np.ndarray:
        return self.centre + self.normal * self.half_l[:, None]

    @property
    def right(self) -> np.ndarray:
        return self.centre - self.normal * self.half_r[:, None]

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def project(self, xy, near: int | None = None, window: int = 80) -> tuple[int, float]:
        """点投影到中心线:返回 (最近点下标, 横向偏移左正)。

        near 给上一帧的下标就只在前后 window 个点里找 —— 掉头之后两段路可能只隔
        一个路宽,全局找最近点会认到隔壁那段。
        """
        lo, hi = (0, len(self.centre)) if near is None else (
            max(0, near - window), min(len(self.centre), near + window + 1))
        d = self.centre[lo:hi] - np.asarray(xy, float)
        i = lo + int(np.argmin(np.einsum('ij,ij->i', d, d)))
        rel = np.asarray(xy, float) - self.centre[i]
        n = (-math.sin(self.heading[i]), math.cos(self.heading[i]))
        return i, float(rel[0] * n[0] + rel[1] * n[1])


def inside_many(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """一批点 (n, 2) 是否在闭合多边形内(射线法)。"""
    a, b = poly[None], np.roll(poly, -1, axis=0)[None]
    x, y = pts[:, :1], pts[:, 1:]
    cross = (a[..., 1] > y) != (b[..., 1] > y)
    with np.errstate(divide='ignore', invalid='ignore'):
        xi = a[..., 0] + (y - a[..., 1]) * (b[..., 0] - a[..., 0]) / (b[..., 1] - a[..., 1])
    return np.count_nonzero(cross & (x < xi), axis=1) % 2 == 1


def inside(poly: np.ndarray, xy) -> bool:
    """点是否在闭合多边形内(射线法)。"""
    x, y = xy
    a, b = poly, np.roll(poly, -1, axis=0)
    cross = (a[:, 1] > y) != (b[:, 1] > y)
    with np.errstate(divide='ignore', invalid='ignore'):
        xi = a[:, 0] + (y - a[:, 1]) * (b[:, 0] - a[:, 0]) / (b[:, 1] - a[:, 1])
    return bool(np.count_nonzero(cross & (x < xi)) % 2)


# ---- 生成 ----
def _elements(rng: np.random.Generator, lv: Level) -> list[dict]:
    """元素序列。每个元素:kind + 参数。首尾各一段直道(起步 / 终点区)。"""
    width = rng.uniform(*lv.width)
    out = [dict(kind='straight', length=rng.uniform(2.0, 3.0), width=width)]

    def straight():
        if rng.random() < lv.gap_chance:
            # 无边界段只放在直道上,前后各留 1m 看得见的边界
            out.append(dict(kind='straight', length=1.0, width=width))
            out.append(dict(kind='gap', length=rng.uniform(*lv.gap_len), width=width))
            out.append(dict(kind='straight', length=1.0, width=width))
        else:
            out.append(dict(kind='straight', length=rng.uniform(*lv.straight), width=width))

    def r_in():
        return rng.uniform(0.0, 0.1) if rng.random() < lv.sharp_chance else rng.uniform(*lv.r_in)

    for _ in range(int(rng.integers(lv.n_elems[0], lv.n_elems[1] + 1))):
        sign = 1 if rng.random() < 0.5 else -1
        u = rng.random()
        if u < lv.uturn_chance:
            out.append(dict(kind='arc', angle=180.0 * sign, r_in=rng.uniform(*lv.uturn_r_in),
                            width=width))
        elif u < lv.uturn_chance + lv.jog_chance:
            out.append(dict(kind='arc', angle=90.0 * sign, r_in=r_in(), width=width))
            out.append(dict(kind='straight', length=rng.uniform(1.2, 3.0), width=width))
            out.append(dict(kind='arc', angle=-90.0 * sign, r_in=r_in(), width=width))
        elif u < lv.uturn_chance + lv.jog_chance + lv.taper_chance:
            new = rng.uniform(*lv.width)
            out.append(dict(kind='taper', length=rng.uniform(1.5, 3.0), width=width, to=new))
            width = new
        else:
            out.append(dict(kind='arc', angle=float(rng.choice(lv.angles)) * sign, r_in=r_in(),
                            width=width))
        straight()
    out.append(dict(kind='straight', length=3.0, width=width))
    return out


def _gap_elements(rng: np.random.Generator, lv: Level) -> list[dict]:
    """L6:前面一段 S 弯/台阶或直道,紧接一个开口方角。重复 2~4 次,方向随机。"""
    width = rng.uniform(*lv.width)
    out = [dict(kind='straight', length=rng.uniform(2.0, 3.0), width=width)]
    for _ in range(int(rng.integers(2, 5))):
        u = rng.random()
        if u < 0.4:
            a = float(rng.choice([45.0, 90.0])) * (1 if rng.random() < 0.5 else -1)
            out.append(dict(kind='arc', angle=a, r_in=rng.uniform(0.3, 1.2), width=width))
            out.append(dict(kind='straight', length=rng.uniform(0.5, 2.5), width=width))
        elif u < 0.7:
            # 缓 S 弯直接进方角:车不用减速,满速冲到角前(赛道 A 左上角,cap 1.0 时 12% 撞在这,
            # 原来方角前只有急弯或直道,2026-09-26)
            a = float(rng.uniform(20.0, 45.0)) * (1 if rng.random() < 0.5 else -1)
            out.append(dict(kind='arc', angle=a, r_in=rng.uniform(1.5, 3.0), width=width))
            out.append(dict(kind='arc', angle=-a, r_in=rng.uniform(1.5, 3.0), width=width))
            out.append(dict(kind='straight', length=rng.uniform(*(lv.gap_lead or (1.0, 3.0))),
                            width=width))
        else:
            out.append(dict(kind='straight', length=rng.uniform(*(lv.gap_lead or (1.5, 4.0))),
                            width=width))
        sign = 1 if rng.random() < 0.5 else -1
        out.append(dict(kind='arc', angle=90.0 * sign, r_in=rng.uniform(0.0, 0.1), width=width,
                        # 外角方角的桶距 0.4~1.5 全覆盖:只练稀桶时车学会了"方角 = 稀桶",
                        # 赛道 C 左上角那种密桶方角(0.5m)反而一头撞上去(v2_d,2026-09-26)
                        gap_corner=rng.random() < 0.9, gap_spacing=rng.uniform(*lv.gap_spacing),
                        gap_phase=rng.uniform(0.0, 1.0),
                        decoy=rng.random() < lv.decoy_chance, decoy_kind=int(rng.integers(3)),
                        decoy_dist=rng.uniform(0.8, 4.0), decoy_len=rng.uniform(2.0, 6.0),
                        decoy_spacing=rng.uniform(0.5, 1.5), decoy_seed=int(rng.integers(2**31))))
        out.append(dict(kind='straight', length=rng.uniform(1.5, 3.5), width=width))
    out.append(dict(kind='straight', length=3.0, width=width))
    return out


def _materials(rng: np.random.Generator, lv: Level, elems: list[dict]) -> None:
    """每个元素两侧各选一种材质。整条路先抽一个"风格",元素之间小概率换。"""
    style = [WALL if rng.random() < lv.wall_chance else CONE for _ in range(2)]
    if lv.edge:
        style = [EDGE, EDGE]
    for e in elems:
        if e['kind'] == 'gap':
            e['mat'] = (NONE, NONE)
            continue
        side = []
        for k in range(2):
            if rng.random() < 0.15 and not lv.edge:
                style[k] = WALL if style[k] == CONE else CONE
            side.append(style[k])
        if e['kind'] == 'straight' and rng.random() < lv.one_side_none_chance:
            side[int(rng.integers(2))] = NONE
        e['mat'] = tuple(side)


def build(elems: list[dict], cone_spacing: float = 0.7, cone_radius: float = 0.14) -> Road:
    x = y = yaw = 0.0
    pts, hs, half, ml, mr = [], [], [], [], []
    for e in elems:
        w0 = e['width']
        e['_i0'] = len(pts)
        if e['kind'] == 'arc':
            rc = e['r_in'] + w0 / 2
            a = math.radians(abs(e['angle']))
            sign = 1.0 if e['angle'] > 0 else -1.0
            n = max(2, int(math.ceil(rc * a / DS)))
            cx, cy = x - sign * rc * math.sin(yaw), y + sign * rc * math.cos(yaw)
            for k in range(n):
                t = yaw + sign * a * k / n
                pts.append((cx + sign * rc * math.sin(t), cy - sign * rc * math.cos(t)))
                hs.append(t)
                half.append(w0 / 2)
            yaw += sign * a
            x, y = cx + sign * rc * math.sin(yaw), cy - sign * rc * math.cos(yaw)
        else:
            n = max(1, int(math.ceil(e['length'] / DS)))
            w1 = e.get('to', w0)
            for k in range(n):
                f = k / n
                pts.append((x + math.cos(yaw) * e['length'] * f, y + math.sin(yaw) * e['length'] * f))
                hs.append(yaw)
                half.append((w0 + (w1 - w0) * f) / 2)
            x += math.cos(yaw) * e['length']
            y += math.sin(yaw) * e['length']
        ml += [e['mat'][0]] * n
        mr += [e['mat'][1]] * n
    pts.append((x, y))
    hs.append(yaw)
    half.append(half[-1])
    ml.append(ml[-1])
    mr.append(mr[-1])
    centre = np.array(pts)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(centre, axis=0).T))])
    road = Road(centre, np.unwrap(np.array(hs)), s, np.array(half), np.array(ml), np.array(mr),
                cone_radius=cone_radius, elems=elems)
    road.walls, road.cones = _boundary_objects(road, cone_spacing)
    for i, e in enumerate(elems):
        e['_i1'] = elems[i + 1]['_i0'] if i + 1 < len(elems) else len(centre) - 1
        if e.get('gap_corner'):
            _square_gap_corner(road, e)
    return road


def _square_gap_corner(road: Road, e: dict) -> None:
    """把尖 90° 弯的外侧锥桶换成方角:两条外边线延长交于顶点,从顶点起按稀间距排桶。

    相位随机:相位 0 时顶点上正好一个桶(赛道 A 左上角就是这样),
    正前方(来车方向的延长线)落在两桶之间的缺口里。
    """
    sign = 1.0 if e['angle'] > 0 else -1.0
    i0, i1 = e['_i0'], e['_i1']
    side = road.right if sign > 0 else road.left                # 左转的外侧是右边
    mat = road.mat_right if sign > 0 else road.mat_left
    if mat[i0] != CONE:
        return
    h0, h1 = road.heading[i0], road.heading[i1]
    d0, d1 = np.array([math.cos(h0), math.sin(h0)]), np.array([math.cos(h1), math.sin(h1)])
    p0, p1 = side[i0], side[i1]
    # p0 + a*d0 = p1 - b*d1
    A = np.array([d0, d1]).T
    a, b = np.linalg.solve(A, p1 - p0)
    vertex = p0 + a * d0
    # 去掉这个弯前后 2m 内、外侧的旧桶
    reach = 2.0
    lo = max(0, int(np.searchsorted(road.s, road.s[i0] - reach)))
    hi = min(len(road.s) - 1, int(np.searchsorted(road.s, road.s[i1] + reach)))
    nrm = road.normal
    keep = []
    for c in road.cones:
        j = lo + int(np.argmin(np.hypot(*(road.centre[lo:hi + 1] - c).T)))
        near = np.hypot(*(road.centre[j] - c)) < road.half[j] + 0.4
        outer = np.dot(c - road.centre[j], nrm[j]) * sign < 0
        keep.append(not (lo < j < hi and near and outer))
    sp, ph = e['gap_spacing'], e['gap_phase'] * e['gap_spacing']
    new = []
    for d, leg in ((-d0, a + reach), (d1, b + reach)):     # 顶点往来车方向退 / 往出弯方向走
        t = ph if (ph > 1e-6 or d is not d1) else sp      # 相位 0 时顶点上的桶只放一次
        while t <= leg:
            new.append(vertex + d * t)
            t += sp
    if e.get('decoy'):
        # 外墙后面摆东西:0 顺来车方向的两排(假走廊) 1 横着一排(像远处的墙,
        # 赛道 A 上加这种 v2_h 仍 0.02,只练走廊不够) 2 随手散一堆
        q0 = (road.left if sign > 0 else road.right)[i0]
        mid, dist, ln, sp_ = (p0 + q0) / 2, e['decoy_dist'], e['decoy_len'], e['decoy_spacing']
        if e['decoy_kind'] == 0:
            for p in (p0, q0):
                new += [p + d0 * t for t in np.arange(a + dist, a + dist + ln, sp_)]
        elif e['decoy_kind'] == 1:
            c = mid + d0 * (a + dist)
            new += [c + d1 * t for t in np.arange(-ln / 2, ln / 2, sp_)]
        else:
            r = np.random.default_rng(e['decoy_seed'])
            for _ in range(int(r.integers(5, 16))):
                new.append(mid + d0 * (a + r.uniform(0.8, 0.8 + dist + 1)) + d1 * r.uniform(-2.5, 2.5))
    road.cones = np.vstack([road.cones[np.array(keep, bool)], np.array(new)])


def _runs(mat: np.ndarray, kind: int):
    """材质数组里连续等于 kind 的 [lo, hi) 区间。"""
    on = np.r_[False, mat == kind, False]
    edges = np.flatnonzero(np.diff(on.astype(int)))
    return zip(edges[::2], edges[1::2])


def _resample(poly: np.ndarray, step: float) -> tuple[np.ndarray, np.ndarray]:
    """折线按弧长 step 重采样,返回 (点, 每点弧长)。重合点(尖内角)自然被跳过。"""
    seg = np.hypot(*np.diff(poly, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return poly[:1], s[:1]
    t = np.linspace(0, s[-1], max(2, int(math.ceil(s[-1] / step)) + 1))
    return np.stack([np.interp(t, s, poly[:, 0]), np.interp(t, s, poly[:, 1])], axis=1), t


def _boundary_objects(road: Road, cone_spacing: float):
    walls, cones = [], []
    for side, mat in ((road.left, road.mat_left), (road.right, road.mat_right)):
        for lo, hi in _runs(mat, WALL):
            p, _ = _resample(side[lo:hi + 1 if hi < len(side) else hi], WALL_STEP)
            seg = np.stack([p[:-1], p[1:]], axis=1)
            walls.append(seg[np.hypot(*(seg[:, 1] - seg[:, 0]).T) > 1e-6])
        for lo, hi in _runs(mat, CONE):
            p, _ = _resample(side[lo:hi], cone_spacing)
            cones.append(p)
    walls = np.concatenate(walls) if walls else np.zeros((0, 2, 2))
    cones = np.concatenate(cones) if cones else np.zeros((0, 2))
    if len(cones) > 1:          # 尖内角处两段的锥桶会叠在一起,去重
        keep = [0]
        for i in range(1, len(cones)):
            if np.min(np.hypot(*(cones[keep] - cones[i]).T)) > 0.3 * cone_spacing:
                keep.append(i)
        cones = cones[keep]
    return walls, cones


def self_overlaps(road: Road, clearance: float = 0.4) -> bool:
    """路和自己隔得太近(掉头后两段贴在一起、绕回来压到前面)就算重叠。

    弧长隔得远、欧氏距离却近于 两边半宽 + clearance 的点对就是重叠。
    """
    idx = np.arange(0, len(road.s), 5)
    c, s, h = road.centre[idx], road.s[idx], road.half[idx]
    d = np.hypot(c[:, None, 0] - c[None, :, 0], c[:, None, 1] - c[None, :, 1])
    need = h[:, None] + h[None, :] + clearance
    far = np.abs(s[:, None] - s[None, :]) > 2.0 * need
    return bool(np.any(far & (d < need)))


OBST_START, OBST_END = 2.5, 2.0       # 起点后、终点前这么远内不摆障碍 m
OBST_SPACING = 2.5                     # 两个障碍沿路至少隔这么远 m
OBST_PASS = 0.65                       # 绕行那一侧至少留这么宽 m(车宽 0.35 + 两侧墙余量,再留一点)
OBST_LONG = 0.3                        # 规划可行域收窄的那一截前后再各多算这么长(车身有长度)


def add_obstacles(road: Road, rng: np.random.Generator, lv: Level) -> None:
    """按 lv.obstacles 在路面上摆障碍(就地改 road)。每个障碍随机挑一侧留出绕行空间,另一侧可以贴边。"""
    if lv.obstacles[1] <= 0:                            # 不抽随机数:别的级别生成的路逐位不变(测试集指纹)
        return
    n = int(rng.integers(lv.obstacles[0], lv.obstacles[1] + 1))
    if n <= 0 or road.length < OBST_START + OBST_END + 0.5:
        return
    hl, hr = road.half_l.copy(), road.half_r.copy()
    ml, mr = np.asarray(road.mat_left).copy(), np.asarray(road.mat_right).copy()
    boxes, used = [], []
    for _ in range(n * 10):
        if len(boxes) == n:
            break
        s0 = rng.uniform(OBST_START, road.length - OBST_END)
        if any(abs(s0 - u) < OBST_SPACING for u in used):
            continue
        i = int(np.searchsorted(road.s, s0))
        w, depth = rng.uniform(*lv.obstacle_w), rng.uniform(*lv.obstacle_l)
        go_left = rng.random() < 0.5                    # 从左边绕
        room = hl[i] + hr[i] - w - OBST_PASS             # 障碍能横向挪动的余地
        if room < 0:
            continue
        # 横向位置:障碍靠被挡那一侧,离那侧边界 0 ~ room(0 = 贴边)
        o = (-hr[i] + w / 2 + rng.uniform(0, room)) if go_left else (hl[i] - w / 2 - rng.uniform(0, room))
        c, nrm = road.centre[i], road.normal[i]
        t = np.array([nrm[1], -nrm[0]])                  # 路的前进方向
        ctr = c + nrm * o
        corners = np.array([ctr + t * a * depth / 2 + nrm * b * w / 2 for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        span = (road.s >= s0 - depth / 2 - OBST_LONG) & (road.s <= s0 + depth / 2 + OBST_LONG)
        if go_left:
            hr[span] = np.minimum(hr[span], -(o + w / 2))
            mr[span] = WALL
        else:
            hl[span] = np.minimum(hl[span], o - w / 2)
            ml[span] = WALL
        boxes.append(corners)
        used.append(s0)
    if not boxes:
        return
    road.half_left, road.half_right, road.mat_left, road.mat_right = hl, hr, ml, mr
    road.obstacles = np.asarray(boxes, float)
    segs = []
    for b in road.obstacles:
        for k in range(4):
            p, _ = _resample(np.array([b[k], b[(k + 1) % 4]]), WALL_STEP)
            segs.append(np.stack([p[:-1], p[1:]], axis=1))
    road.walls = np.concatenate([road.walls.reshape(-1, 2, 2)] + segs)


def _line_x(p, d, q, e):
    """直线 p + t·d 和 q + u·e 的交点。"""
    t, _ = np.linalg.solve(np.array([d, -e]).T, q - p)
    return p + t * d


def add_pockets(road: Road, rng: np.random.Generator, lv: Level) -> None:
    """按 lv.pocket_chance 把有墙的尖 90° 弯做成口袋(就地改 road):出弯那条走廊往反方向伸出 pocket_depth 再封墙。
    拐角外侧的圆角墙换成方角;口袋压到别的路段就不做。"""
    if lv.pocket_chance <= 0:                           # 不抽随机数:别的级别逐位不变
        return
    polys, add, drop = [], [], np.zeros(len(road.walls), bool)
    for e in road.elems:
        if e['kind'] != 'arc' or abs(e['angle']) != 90.0 or e['r_in'] > 0.1:
            continue
        if rng.random() >= lv.pocket_chance:
            continue
        depth = rng.uniform(*lv.pocket_depth)
        sign = 1.0 if e['angle'] > 0 else -1.0
        i0, i1 = e['_i0'], e['_i1']
        outer, mat = (road.right, road.mat_right) if sign > 0 else (road.left, road.mat_left)
        inner = road.left if sign > 0 else road.right
        lo, hi = max(0, i0 - 2), min(len(mat) - 1, i1 + 2)
        if not np.all(np.asarray(mat)[lo:hi + 1] == WALL):
            continue
        d0 = np.array([math.cos(road.heading[i0]), math.sin(road.heading[i0])])
        d1 = np.array([math.cos(road.heading[i1]), math.sin(road.heading[i1])])
        p0, p1 = outer[i0], outer[i1]
        a = _line_x(p0, d0, inner[i1], d1)               # 来车外侧墙 × 出弯内侧墙:口袋开口的一角
        b = _line_x(p0, d0, p1, d1)                      # 方角顶点
        a2, b2 = a - depth * d1, b - depth * d1
        poly = np.array([p0, a, a2, b2, p1])
        # 口袋不能压到别的路段(弧长隔得远、离口袋近于半宽 + 0.4)
        far = np.abs(road.s - road.s[i0]) > 3.0
        probe = np.vstack([poly, (a2 + b2) / 2, (a + b2) / 2])
        dist = np.hypot(road.centre[far, None, 0] - probe[None, :, 0], road.centre[far, None, 1] - probe[None, :, 1])
        if far.any() and np.any(dist < road.half[far, None] + 0.4):
            continue
        # 拆掉外侧圆角墙(端点落在圆角上的段),换成方角 + 口袋三面墙;两头各多铺一格接上原来的直墙
        rc = e['r_in'] + e['width'] / 2
        c = road.centre[i0] + sign * rc * road.normal[i0]
        r_out = e['r_in'] + e['width']
        w = road.walls.reshape(-1, 2) - c
        on_arc = ((np.abs(np.hypot(w[:, 0], w[:, 1]) - r_out) < 0.05) & (w @ d0 > 1e-3) & (w @ d1 < -1e-3))
        drop |= on_arc.reshape(-1, 2).any(axis=1)
        for q0, q1 in ((p0 - WALL_STEP * d0, a), (a, a2), (a2, b2), (b2, p1 + WALL_STEP * d1)):
            pts, _ = _resample(np.array([q0, q1]), WALL_STEP)
            add.append(np.stack([pts[:-1], pts[1:]], axis=1))
        polys.append(poly)
        e['pocket'] = depth
    if not polys:
        return
    road.walls = np.concatenate([road.walls[~drop]] + add)
    road.pockets = np.asarray(polys, float)


def generate(rng: np.random.Generator, lv: Level, tries: int = 50) -> Road:
    """按 lv 生成一条不自重叠的路。可通性另外查(road_check)。"""
    for _ in range(tries):
        elems = _gap_elements(rng, lv) if lv.gap_corners else _elements(rng, lv)
        _materials(rng, lv, elems)
        road = build(elems, cone_spacing=rng.uniform(*lv.cone_spacing))
        if not self_overlaps(road):
            add_pockets(road, rng, lv)
            add_obstacles(road, rng, lv)
            return road
    raise RuntimeError(f'{lv.name}: {tries} 次都生成了自重叠的路,参数范围有问题')


# ---- 闭环赛道 ----
def _loop_cones(side: np.ndarray, kappa: np.ndarray, is_left: bool, sp: tuple) -> np.ndarray:
    """沿闭合边界按位置变间距排锥桶:直道 / 弯内侧 / 弯外侧。kappa 是中心线曲率(左正)。"""
    closed = np.vstack([side, side[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    straight, inner, outer = sp
    out, t = [], 0.0
    while t < s[-1] - 0.2:
        i = min(int(np.searchsorted(s, t, side='right')) - 1, len(side) - 1)
        out.append(closed[i] + (closed[i + 1] - closed[i]) * (t - s[i]) / max(seg[i], 1e-9))
        k = kappa[i]
        if abs(k) < 0.25:
            t += straight
        else:
            t += inner if (k > 0) == is_left else outer
    return np.array(out)


def generate_loop(rng: np.random.Generator, lv: Level, laps: float = 1.0) -> Road:
    """按规则拼一条闭环锥桶赛道,从随机位置展开成 laps 圈的 Road(再多 2m 当终点区)。

    车在任何位置都能看到折回来的其它路段的锥桶 —— 这正是开放路段里没有、
    赛道 A 上把 v2 干扰掉的东西(2026-09-25)。
    """
    from dataclasses import replace as _replace
    from common.params import RuleTrackParams
    from simulation.fastsim.track_rules import make_rule_track
    from training.road_teacher import curvature
    width = rng.uniform(*lv.loop_width)
    bend = rng.uniform(*lv.loop_bend)
    # 形状旋钮:None 就整条不传,RuleTrackParams 的默认值原样生效 —— 老配置逐位不变。
    # 传了就抽一次;末三个参数在 _layout 里是逐元素抽的,所以这里抽到的是"整条赛道"的设定。
    def _pick(name, default):
        r = getattr(lv, name)
        if r is None:
            return default
        return rng.uniform(*r)
    shape = {}
    if lv.loop_height is not None:
        shape['height_min'] = shape['height_max'] = _pick('loop_height', None)
    if lv.loop_square_end is not None:
        shape['square_end_chance'] = _pick('loop_square_end', None)
    if lv.loop_right_angle is not None:
        shape['right_angle_ratio'] = _pick('loop_right_angle', None)
    if lv.loop_s_bends is not None:
        n = int(round(_pick('loop_s_bends', None)))
        shape['s_bends_min'] = shape['s_bends_max'] = n
    if lv.loop_main is not None:
        shape['main_straight_min'], shape['main_straight_max'] = lv.loop_main
    if lv.loop_turn_angles is not None:
        shape['turn_angles'] = tuple(lv.loop_turn_angles)
    p = _replace(RuleTrackParams(), width=width, bend_radius_min=max(width / 2, bend - 0.25),
                 bend_radius_max=1.8 if lv.loop_bend_max is None else lv.loop_bend_max, n_boards=5,
                 end_radius=(width / 2, width / 2 + 0.25) if rng.random() < lv.sharp_end_chance else None,
                 **shape)
    # 上限留规则默认 1.8:全部弯都压到 bend 附近时台阶/S 弯挤成一团,本车(R>=0.83m)过不去,
    # 作废率 34~44%(2026-09-25 试过)。只把下限放低,偶尔出尖角。
    tr = make_rule_track(rng, p)
    c = tr.centerline
    boards = tr.boards
    if rng.random() < 0.5:                          # 一半镜像:顺时针/逆时针都见
        c = (c * (1, -1))[::-1].copy()
        boards = boards * (1, -1)
    kap = curvature(np.vstack([c[-6:], c, c[:6]]), gap=6)[6:-6]
    d = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    nrm = np.stack([-np.sin(h), np.cos(h)], axis=1)
    left, right = c + nrm * width / 2, c - nrm * width / 2
    sp = (rng.uniform(*lv.straight_spacing), rng.uniform(*lv.inner_spacing),
          rng.uniform(*lv.outer_spacing))
    cones = np.vstack([_loop_cones(left, kap, True, sp), _loop_cones(right, kap, False, sp)])
    # 从随机位置展开
    i0 = int(rng.integers(len(c)))
    c = np.roll(c, -i0, axis=0)
    n = int(len(c) * laps) + int(2.0 / DS)
    reps = np.arange(n) % len(c)
    cc = c[reps]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(cc, axis=0).T))])
    dd = np.gradient(cc, axis=0)
    heading = np.unwrap(np.arctan2(dd[:, 1], dd[:, 0]))
    mat = np.full(n, CONE)
    road = Road(cc, heading, s, np.full(n, width / 2), mat, mat.copy(),
                elems=[dict(kind='loop', width=width, bend=bend, spacing=sp)])
    road.cones = cones
    road.walls = np.asarray(boards, float).reshape(-1, 2, 2)     # [规则] 5 块蓝挡板,当墙打
    return road
