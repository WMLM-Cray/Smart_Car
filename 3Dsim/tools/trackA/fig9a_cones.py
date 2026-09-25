"""Cone centres read individually from the 2026 rules' embedded Fig. 9 A image.

Source image: docs/track/fig9A_source.jpg (1350 x 649 pixels). The 16.8 m
horizontal dimension runs from the outer-left blue cone at x=14 to the
outer-right blue cone at x=1335. Pixel placement is image-derived, rather
than a claim of survey-grade field coordinates.
"""

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SOURCE_IMAGE = ROOT / "docs/track/fig9A_source.jpg"
PX_PER_M = (1335 - 14) / 16.8

# Counterclockwise from the left end of the bottom straight. Coordinates were
# checked against every visible hollow cone circle in the original raster.
BLUE = [
    (231,508,"E1"),(319,508,"E1"),(408,508,"E1"),(496,508,"E1"),
    (587,510,"E1"),(673,510,"E1"),(765,509,"E1"),(851,510,"E1"),
    (942,510,"E1"),(1029,508,"E1"),(1119,508,"E1"),
    (1201,491,"E2"),(1271,445,"E2"),(1318,375,"E2"),(1335,292,"E2"),
    (1318,209,"E2"),(1271,139,"E2"),(1202,92,"E2"),(1120,76,"E2"),
    (674,76,"E5"),(592,92,"E5"),(522,140,"E5"),
    (490,160,"E6"),(453,168,"E6"),(415,161,"E6"),(383,140,"E6"),
    (313,92,"E7"),(231,76,"E7"),(115,76,"E8"),(14,76,"E8"),
    (14,184,"E9"),(14,242,"E9"),(14,292,"E9"),
    (31,375,"E10"),(78,445,"E10"),(148,491,"E10"),
]

RED = [
    (231,390,"I1"),(319,390,"I1"),(408,390,"I1"),(496,390,"I1"),
    (585,390,"I1"),(674,390,"I1"),(762,389,"I1"),(851,390,"I1"),
    (940,390,"I1"),(1029,390,"I1"),(1119,390,"I1"),
    (1156,383,"I2"),(1188,361,"I2"),(1211,327,"I2"),(1217,292,"I2"),
    (1209,255,"I2"),(1188,222,"I2"),(1156,201,"I2"),(1121,194,"I2"),
    (675,194,"I5"),(637,202,"I5"),(605,223,"I5"),
    (534,269,"I6"),(453,286,"I6"),(370,270,"I6"),(301,224,"I6"),
    (267,201,"I7"),(229,194,"I7"),(180,194,"I8"),(132,194,"I8"),
    (132,242,"I9"),(132,292,"I9"),
    (140,329,"I10"),(161,361,"I10"),(193,383,"I10"),
]


def pixel_to_metres(px, py):
    return (round((px - 14) / PX_PER_M, 6), round((508 - py) / PX_PER_M, 6))


def rows():
    for color, prefix, points in (("blue", "b", BLUE), ("red", "r", RED)):
        for i, (px, py, segment) in enumerate(points, 1):
            x, y = pixel_to_metres(px, py)
            yield {"id": f"{prefix}{i:02d}", "color": color, "x": x, "y": y,
                   "segment": segment, "source_px_x": px, "source_px_y": py}


def write_csv(path):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["id", "color", "x", "y", "segment"],
                                lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows())


def write_overlay(path):
    from PIL import Image, ImageDraw, ImageFont

    image = Image.open(SOURCE_IMAGE).convert("RGB")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("arial.ttf", 13)
    except OSError:
        font = ImageFont.load_default()
    for row in rows():
        px, py = row["source_px_x"], row["source_px_y"]
        ink = "#063e9d" if row["color"] == "blue" else "#aa1424"
        draw.ellipse((px-4,py-4,px+4,py+4),fill=ink,outline="white",width=1)
        draw.text((px+8,py-20 if row["color"] == "blue" else py+5),row["id"],
                  fill=ink,stroke_width=2,stroke_fill="white",font=font)
    image.save(path)


if __name__ == "__main__":
    output = ROOT / "docs/track"
    write_csv(output / "trackA_cones.csv")
    write_overlay(output / "trackA_fig9_cone_overlay.png")
    print(f"{len(BLUE)} blue / {len(RED)} red from {SOURCE_IMAGE}")
