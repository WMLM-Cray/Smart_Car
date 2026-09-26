"""Draw a standalone Track A plan from the generated cone CSV, without a source-image background."""

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[3]
CSV = ROOT / "maps/trackA_cones.csv"
OUTPUT = ROOT / "maps/trackA.png"
SCALE = 82
LEFT, TOP = 100, 112
HEIGHT = 720
Y_TOP = 6.1


def xy(x, y):
    return (round(LEFT + x * SCALE), round(TOP + (Y_TOP - y) * SCALE))


def from_source(px, py):
    # Only the official 16.8 m horizontal scale is used; no raster is loaded.
    return xy((px - 14) * 16.8 / 1321, (508 - py) * 16.8 / 1321)


def font(size):
    for name in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
                 "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def dashed(draw, a, b, color, width=3, dash=9, gap=7):
    length = math.dist(a, b)
    if length == 0:
        return
    t = 0
    while t < length:
        u = min(t + dash, length)
        draw.line(((a[0] + (b[0] - a[0]) * t / length,
                    a[1] + (b[1] - a[1]) * t / length),
                   (a[0] + (b[0] - a[0]) * u / length,
                    a[1] + (b[1] - a[1]) * u / length)), fill=color, width=width)
        t += dash + gap


def render(output=OUTPUT):
    with CSV.open(newline="", encoding="utf-8") as stream:
        cones = list(csv.DictReader(stream))
    assert len(cones) == 71, "Expected the 71 illustrated Fig. 9A cones"
    im = Image.new("RGB", (1600, HEIGHT), "#f7f9fc")
    d = ImageDraw.Draw(im)
    title, label, small = font(30), font(20), font(15)
    blue, red, yellow = "#2563eb", "#e64b53", "#e2ad08"

    # Road features, placed in the same metre frame as the cone CSV.
    for a, b, name, color in ((231, 408, "红绿灯示例", "#12a9bc"),
                               (496, 585, "A", "#dfb71b"),
                               (674, 762, "A/B", "#dfb71b"),
                               (851, 940, "发车区", "#dfb71b")):
        x0, y0 = from_source(a, 508)
        x1, y1 = from_source(b, 390)
        d.rectangle((x0, y1, x1, y0), outline=color, width=3)
        d.text((x0 + 6, y0 - 33), name, fill="#334155", font=small)

    # The yellow tape V replaces a long cone-free gap in each cone ring.
    for points in (((674, 76), (895, 169), (1120, 76)),
                   ((675, 194), (895, 286), (1121, 194))):
        d.line([from_source(*p) for p in points], fill=yellow, width=6, joint="curve")

    for color, ink, excluded in (("blue", blue, ("E2", "E5")),
                                 ("red", red, ("I2", "I5"))):
        ring = [r for r in cones if r["color"] == color]
        for a, b in zip(ring, ring[1:] + ring[:1]):
            if (a["segment"], b["segment"]) != excluded:
                dashed(d, xy(float(a["x"]), float(a["y"])),
                       xy(float(b["x"]), float(b["y"])), ink)
        for cone in ring:
            x, y = xy(float(cone["x"]), float(cone["y"]))
            d.ellipse((x - 9, y - 9, x + 9, y + 9), fill="white", outline=ink, width=4)

    d.text((LEFT, 30), "赛道 A · 生成俯视图", fill="#12243d", font=title)
    d.text((LEFT + 445, 40), "蓝桶 36   红桶 35   黄色 V 为胶带（无桶）",
           fill="#42546c", font=label)
    d.text((LEFT, HEIGHT - 40), "按规则图逐桶读点并按 16.8 m 标注缩放；图片仅由坐标绘制。",
           fill="#56657a", font=small)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    im.save(output)
    print(output)


if __name__ == "__main__":
    render()
