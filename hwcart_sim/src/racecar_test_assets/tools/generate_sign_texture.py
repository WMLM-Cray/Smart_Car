"""Generate the sign face textures (white background, black letter).

Self-generated, no external asset, no license burden. Deterministic: fixed
canvas size, fixed colours, fixed letter. Writes PNG files next to the sign
material scripts so the package stays self-contained:

    models/sign_a/materials/textures/sign_a.png   (letter "A")
    models/sign_b/materials/textures/sign_b.png   (letter "B")

Requires Pillow (PIL). A Truetype font is used when available
(DejaVuSans-Bold / LiberationSans-Bold, standard on Ubuntu 22.04); otherwise
PIL's tiny built-in bitmap font is used as a fallback (still legible, smaller).

Usage: python3 tools/generate_sign_texture.py
"""
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
OUT = {
    'sign_a': PKG_ROOT / 'models' / 'sign_a' / 'materials' / 'textures' / 'sign_a.png',
    'sign_b': PKG_ROOT / 'models' / 'sign_b' / 'materials' / 'textures' / 'sign_b.png',
}
SIZE = 512
BG = (255, 255, 255)
FG = (0, 0, 0)


def _load_font(size):
    try:
        from PIL import ImageFont
        for p in (
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
            '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
        ):
            if Path(p).is_file():
                return ImageFont.truetype(p, size)
    except Exception:
        pass
    try:
        from PIL import ImageFont
        return ImageFont.load_default()
    except Exception:
        return None


def make(path, letter):
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (SIZE, SIZE), BG)
    d = ImageDraw.Draw(img)
    font = _load_font(int(SIZE * 0.85))
    try:
        bbox = d.textbbox((0, 0), letter, font=font)
    except Exception:
        bbox = None
    if bbox and bbox[2] > bbox[0] and bbox[3] > bbox[1]:
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        x = (SIZE - w) / 2.0 - bbox[0]
        y = (SIZE - h) / 2.0 - bbox[1]
        d.text((x, y), letter, fill=FG, font=font)
    else:
        d.text((SIZE / 2, SIZE / 2), letter, fill=FG, font=font, anchor='mm')
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    print(f'wrote: {path.relative_to(PKG_ROOT).as_posix()} '
          f'({SIZE}x{SIZE}, letter {letter!r})')


def main():
    for sign, letter in (('sign_a', 'A'), ('sign_b', 'B')):
        make(OUT[sign], letter)
    return 0


if __name__ == '__main__':
    sys.exit(main())
