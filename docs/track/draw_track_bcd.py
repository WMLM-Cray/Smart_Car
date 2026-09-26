"""Trace the 2026 rules' Track B/C/D diagrams as labelled top-view previews.

The contours follow Fig. 9 by eye. Only the printed 16.8 m length and 1.5 m
width are treated as dimensions; these previews are not validated cone layouts.
"""

from __future__ import annotations

import math
import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parents[2] / "maps"
BLUE, RED, YELLOW = "#4263cf", "#db6262", "#edc737"


def line(a, b, n=10):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            for i in range(n + 1)]


def cubic(a, b, c, d, n=24):
    points = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        points.append((u**3*a[0] + 3*u*u*t*b[0] + 3*u*t*t*c[0] + t**3*d[0],
                       u**3*a[1] + 3*u*u*t*b[1] + 3*u*t*t*c[1] + t**3*d[1]))
    return points


def arc(cx, cy, radius, first_deg, last_deg, n=45):
    return [(cx + radius * math.cos(math.radians(first_deg + (last_deg-first_deg)*i/n)),
             cy + radius * math.sin(math.radians(first_deg + (last_deg-first_deg)*i/n)))
            for i in range(n + 1)]


def join(*segments):
    result = []
    for segment in segments:
        result.extend(segment if not result else segment[1:])
    return result


def contours(kind):
    if kind == "B":
        outer = join(
            line((230, 350), (947, 350)), arc(947, 175, 175, 90, -90),
            line((947, 0), (769, 67)), line((769, 67), (590, 0)),
            cubic((590, 0), (520, 0), (505, 73), (410, 73)),
            cubic((410, 73), (315, 73), (300, 0), (230, 0)),
            arc(230, 175, 175, -90, -270))
        inner = join(
            line((230, 254), (947, 254)), arc(947, 175, 79, 90, -90),
            line((947, 96), (769, 163)), line((769, 163), (590, 96)),
            cubic((590, 96), (520, 96), (505, 169), (410, 169)),
            cubic((410, 169), (315, 169), (300, 96), (230, 96)),
            arc(230, 175, 79, -90, -270))
        zones = [(445, 518, "A"), (590, 662, "AB"), (735, 805, "START")]
        light = (220, 385, "bottom")
    else:
        left_outer = (arc(230, 175, 175, -90, -270) if kind == "D" else
                      join(line((230, 0), (55, 0)), line((55, 0), (55, 175)),
                           arc(230, 175, 175, 180, 90)))
        left_inner = (arc(230, 175, 79, -90, -270) if kind == "D" else
                      join(line((230, 96), (151, 96)), line((151, 96), (151, 175)),
                           arc(230, 175, 79, 180, 90)))
        outer = join(
            line((230, 350), (590, 350)),
            cubic((590, 350), (680, 350), (680, 276), (770, 276)),
            cubic((770, 276), (860, 276), (860, 350), (950, 350)),
            arc(950, 175, 175, 90, -90), line((950, 0), (590, 0)),
            line((590, 0), (410, 75)), line((410, 75), (230, 0)), left_outer)
        inner = join(
            line((230, 254), (590, 254)),
            cubic((590, 254), (680, 254), (680, 180), (770, 180)),
            cubic((770, 180), (860, 180), (860, 254), (950, 254)),
            arc(950, 175, 79, 90, -90), line((950, 96), (590, 96)),
            line((590, 96), (410, 171)), line((410, 171), (230, 96)), left_inner)
        zones = [(230, 305, "A"), (375, 450, "AB"), (520, 590, "START")]
        light = (790, 950, "top")
    return outer, inner, zones, light


def equidistant(points, gap=32):
    """Place visual cone markers along a sampled contour (spacing is schematic)."""
    out = [points[0]]
    remaining = gap
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        while length >= remaining and length > 0:
            t = remaining / length
            a = (a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1]))
            out.append(a)
            length = math.dist(a, b)
            remaining = gap
        remaining -= length
    return out


def cone_marks(kind, points):
    """The yellow V is a continuous marked region, without cones between shoulders."""
    marks = equidistant(points)
    shoulder_y = 0 if points[0][1] == 350 else 96
    if kind == "B":
        marks = [p for p in marks if not (600 < p[0] < 937 and p[1] < 200)]
        marks += [(590, shoulder_y), (947, shoulder_y)]
    else:
        marks = [p for p in marks if not (240 < p[0] < 580 and p[1] < 180)]
        marks += [(230, shoulder_y), (590, shoulder_y)]
    # The four shoulders have cones in Fig. 9; prioritize them over nearby
    # synthetic interval marks so a 280 mm cone footprint is not doubled.
    shoulders = ([(590, shoulder_y), (947, shoulder_y)] if kind == "B" else
                 [(230, shoulder_y), (590, shoulder_y)])
    order = lambda p: min(range(len(points)), key=lambda i: math.dist(p, points[i]))
    kept = []
    for p in shoulders + sorted(marks, key=order):
        if all(math.dist(p, q) >= 19 for q in kept):
            kept.append(p)
    return sorted(kept, key=order)


def export_data(kind, outer, inner, zones, light):
    """Image-estimated metre coordinates, kept separate from official dimensions."""
    scale = 16.8 / (max(p[0] for p in outer) - min(p[0] for p in outer))
    x0 = min(p[0] for p in outer)

    def metric(p):
        return [round((p[0]-x0)*scale, 4), round((350-p[1])*scale, 4)]

    rings = {"outer": [metric(p) for p in outer],
             "inner": [metric(p) for p in inner]}
    def point_segment_distance(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        denom = dx*dx + dy*dy
        t = 0 if denom == 0 else max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom))
        return math.dist(p, (a[0]+t*dx, a[1]+t*dy))

    clearances = [min(point_segment_distance(p, a, b) for a, b in zip(inner, inner[1:]))*scale
                  for p in outer]
    cones = []
    for color, points, prefix in (("blue", outer, "b"), ("red", inner, "r")):
        for i, p in enumerate(cone_marks(kind, points), 1):
            x, y = metric(p)
            cones.append({"id": f"{prefix}{i:03d}", "color": color, "x": x, "y": y})
    data = {
        "track": kind,
        "status": "image_estimated_preview_not_simulation_validated",
        "source": "2026 ROS car high-education draft rules, Fig. 3 and Fig. 9, PDF pages 4 and 11-12",
        "frame": {"origin": "leftmost_outer_cone_bottom_height", "x": "right", "y": "up", "unit": "m"},
        "rule_dimensions": {"total_x_span_m": 16.8, "corridor_width_m": 1.5,
                            "cone_separation_requirement": "less than 1.5 m except yellow tape region"},
        "image_scale_m_per_px": round(scale, 7),
        "assumptions": {"cone_target_spacing_m": round(32*scale, 3),
                        "unlabelled_curve_controls": "traced by eye from rendered Fig. 9",
                        "zone_positions": "traced by eye from rendered Fig. 9"},
        "outer_to_inner_nearest_distance_range_m": [round(min(clearances), 3), round(max(clearances), 3)],
        "boundaries": rings,
        "cones": cones,
        "zones": [{"name": label, "x_range_m": [metric((x0p, 0))[0], metric((x1p, 0))[0]],
                   "side": "bottom"} for x0p, x1p, label in zones],
        "traffic_light_zone": {"x_range_m": [metric((light[0], 0))[0], metric((light[1], 0))[0]],
                               "side": light[2]},
    }
    (OUT / f"track{kind}_layout.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (OUT / f"track{kind}_cones.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["id", "color", "x", "y"])
        writer.writeheader()
        writer.writerows(cones)
    return data


def svg(kind, outer, inner, zones, light):
    def path(points):
        return "M " + " L ".join(f"{x+65:.1f},{y+140:.1f}" for x, y in points) + " Z"

    bits = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 630" role="img" aria-label="Track {kind} traced from the 2026 rules, schematic">',
            '<rect width="1280" height="630" fill="#fbfcff"/>',
            f'<text x="65" y="65" font-family="sans-serif" font-size="31" font-weight="bold" fill="#15213a">Track {kind} · 2026 rules Fig. 9</text>',
            '<text x="65" y="94" font-family="sans-serif" font-size="17" fill="#52617b">Contour traced from official diagram · 1.5 m corridor width (rule) · local coordinates approximate</text>']
    for x0, x1, label in zones:
        bits.append(f'<rect x="{x0+65}" y="{394 if light[2]=="bottom" else 394}" width="{x1-x0}" height="96" fill="none" stroke="{YELLOW}" stroke-width="3"/>')
        bits.append(f'<text x="{(x0+x1)/2+65:.1f}" y="446" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#806500">{label}</text>')
    lx0, lx1, side = light
    ly = 131 if side == "top" else 386
    bits.append(f'<rect x="{lx0+65}" y="{ly}" width="{lx1-lx0}" height="112" fill="none" stroke="#54bfd0" stroke-width="2" stroke-dasharray="6 5"/>')
    bits.append(f'<text x="{(lx0+lx1)/2+65:.1f}" y="{ly+20}" text-anchor="middle" font-family="sans-serif" font-size="13" fill="#08778a">traffic light</text>')
    for points, color in ((outer, BLUE), (inner, RED)):
        bits.append(f'<path d="{path(points)}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        for x, y in cone_marks(kind, points):
            bits.append(f'<circle cx="{x+65:.1f}" cy="{y+140:.1f}" r="6" fill="white" stroke="{color}" stroke-width="2"/>')
    # Yellow V has no cone markers in the document's example diagrams.
    shoulders = ((590, 947) if kind == "B" else (230, 590))
    valley = 769 if kind == "B" else 410
    vdepth = 67 if kind == "B" else 75
    for y0 in (0, 96):
        bits.append(f'<path d="M {shoulders[0]+65},{y0+140} L {valley+65},{y0+vdepth+140} L {shoulders[1]+65},{y0+140}" fill="none" stroke="{YELLOW}" stroke-width="4"/>')
    bits += ['<path d="M 120,545 H 1185" stroke="#31405f" stroke-width="1.5" marker-start="url(#arrow)" marker-end="url(#arrow)"/>',
             '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto-start-reverse"><path d="M 0,0 L 8,4 L 0,8 Z" fill="#31405f"/></marker></defs>',
             '<text x="653" y="537" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#31405f">16.8 m (rule)</text>',
             '<text x="65" y="595" font-family="sans-serif" font-size="15" fill="#68758e">Cone intervals and unlabelled segment coordinates are image estimates. Not a survey or simulator geometry.</text>',
             '</svg>']
    return "\n".join(bits)


def png(kind, outer, inner, zones, light, destination):
    s = 2
    im = Image.new("RGB", (1280*s, 630*s), "#fbfcff")
    draw = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("arial.ttf", 32*s)
        small = ImageFont.truetype("arial.ttf", 17*s)
    except OSError:
        font = small = ImageFont.load_default()
    def xy(p):
        return ((p[0]+65)*s, (p[1]+140)*s)
    draw.text((65*s, 48*s), f"Track {kind} | 2026 rules Fig. 9", fill="#15213a", font=font)
    draw.text((65*s, 91*s), "Contour traced from official diagram | 1.5 m corridor width (rule) | approximate", fill="#52617b", font=small)
    for x0, x1, label in zones:
        draw.rectangle(((x0+65)*s,394*s,(x1+65)*s,490*s),outline=YELLOW,width=4)
        draw.text((((x0+x1)/2+50)*s,425*s), label, fill="#806500",font=small)
    lx0,lx1,side=light
    ly=131 if side=="top" else 386
    draw.rectangle(((lx0+65)*s,ly*s,(lx1+65)*s,(ly+112)*s),outline="#54bfd0",width=3)
    draw.text(((lx0+68)*s,(ly+10)*s),"traffic light",fill="#08778a",font=small)
    for points,color in ((outer,BLUE),(inner,RED)):
        draw.line([xy(p) for p in points],fill=color,width=4,joint="curve")
        for p in cone_marks(kind, points):
            x,y=xy(p)
            draw.ellipse((x-6*s,y-6*s,x+6*s,y+6*s),fill="white",outline=color,width=3)
    shoulders=(590,947) if kind=="B" else (230,590)
    valley=769 if kind=="B" else 410
    vdepth=67 if kind=="B" else 75
    for y0 in (0,96):
        draw.line([xy((shoulders[0],y0)),xy((valley,y0+vdepth)),xy((shoulders[1],y0))],fill=YELLOW,width=7)
    draw.line((120*s,545*s,1185*s,545*s),fill="#31405f",width=2)
    draw.text((565*s,515*s),"16.8 m (rule)",fill="#31405f",font=small)
    draw.text((65*s,585*s),"Cone intervals and unlabelled coordinates are image estimates; not simulator geometry.",fill="#68758e",font=small)
    im.save(destination)


def main():
    OUT.mkdir(exist_ok=True)
    for kind in "BCD":
        outer, inner, zones, light = contours(kind)
        (OUT / f"track{kind}.svg").write_text(svg(kind, outer, inner, zones, light), encoding="utf-8")
        png(kind, outer, inner, zones, light, OUT / f"track{kind}.png")
        data = export_data(kind, outer, inner, zones, light)
        print(f"Track {kind}: {len(outer)} outer / {len(inner)} inner samples, "
              f"{len(data['cones'])} schematic cones, nearest boundary distances "
              f"{data['outer_to_inner_nearest_distance_range_m']} m")


if __name__ == "__main__":
    main()
