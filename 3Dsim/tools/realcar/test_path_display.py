#!/usr/bin/env python3
# test_path_display.py —— path_display 纯逻辑的行为验证(无 ROS 依赖, 本机可跑)
# 覆盖: ①frame 一致直用 ②frame 不一致走 TF 转换 ③无 TF 拒绝绘制 ④空路径清旧线
#       ⑤无路径消息/TF 缺失不崩(状态行"无数据" + 拒绝绘制不抛)
# "后启动能否收到已有路径"取决于实际 QoS, 代码层不覆盖, 由现场验证记录。
# 用法: python3 test_path_display.py
import math
from types import SimpleNamespace as NS
import path_display


def _pose(x, y):
    return NS(pose=NS(position=NS(x=x, y=y)))


def _path(frame, xy_list):
    return NS(header=NS(frame_id=frame), poses=[_pose(x, y) for x, y in xy_list])


def _tf(tx, ty, yaw):
    def lookup(target, source):
        if (target, source) != ("map", "odom"):
            raise RuntimeError(f"no tf {target}<-{source}")
        return tx, ty, yaw
    return lookup


def main():
    ok = 0

    # ① frame 一致: 直用原始坐标(与地图叠加)
    xy, err = path_display.transform_path_poses(_path("map", [(1, 2), (3, 4)]).poses, "map", _tf(0, 0, 0))
    assert err is None and xy == [(1, 2), (3, 4)], "①frame 一致应直用坐标"
    ok += 1

    # ② frame 不一致 + TF 纯平移(map<-odom = (3,4,0)): 点 (2,0) -> (5,4)
    xy, err = path_display.transform_path_poses(_path("odom", [(2, 0)]).poses, "odom", _tf(3, 4, 0))
    assert err is None and abs(xy[0][0] - 5) < 1e-9 and abs(xy[0][1] - 4) < 1e-9, f"②纯平移错: {xy}"
    ok += 1

    # ③ frame 不一致 + TF 旋转90°+平移(1,0): odom 点 (2,0) -> map 点 (1,2)
    xy, err = path_display.transform_path_poses(_path("odom", [(2, 0)]).poses, "odom", _tf(1, 0, math.pi / 2))
    assert err is None and abs(xy[0][0] - 1) < 1e-9 and abs(xy[0][1] - 2) < 1e-9, f"③旋转90°错: {xy}"
    ok += 1

    # ④ frame 不一致 + TF 不可得: 拒绝绘制(不直接套坐标, 不抛)
    xy, err = path_display.transform_path_poses(_path("base_link", [(1, 1)]).poses, "base_link", _tf(0, 0, 0))
    assert xy == [] and err and "拒绝绘制" in err, "④无 TF 应拒绝绘制"
    ok += 1

    # ⑤ 空路径: 返回空列表(调用方不画, 旧线随 render 每帧重建自然消失)
    xy, err = path_display.transform_path_poses(_path("map", []).poses, "map", _tf(0, 0, 0))
    assert err is None and xy == [], "⑤空路径应返回空列表"
    ok += 1

    # ⑥ 状态行: 未订阅->None; 无数据->"无数据"; 有数据->"最近收到"+年龄, 且无"已加载"字样
    assert path_display.path_status_text(NS(path_topic=None), 0) is None, "⑥未订阅应返回 None"
    assert path_display.path_status_text(NS(path_topic="/plan", path=None), 0) == ("plan: 无数据", "gray")
    st = path_display.path_status_text(NS(path_topic="/plan", path=_path("map", [(0, 0)]), last_path_recv=0.0), 3.0)
    assert "最近收到" in st[0] and "已加载" not in st[0], f"⑥状态行文案错: {st}"
    ok += 1

    print(f"PASS {ok}/6  (path_display 纯逻辑行为验证)")
    print("注: '后启动能否收到已有路径'取决于发布者实际 QoS(TRANSIENT_LOCAL 可收到/ VOLATILE 不可),")
    print("    本单测不覆盖, 留给现场验证并如实记录。")


if __name__ == "__main__":
    main()
