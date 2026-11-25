"""Generate the A3 saturation test sheet (saturation_sheet.svg).

16 equal blocks of solid tone, 8-bit greyscale surface_value from 255 (white)
to 0 (black), spaced more closely at the dark end where a plateau is expected.

Block size: the five sensors span ~64 mm side to side and ~13 mm front to back
(DN1/DN5 sit ~13 mm behind DN2-DN4). A 95 x 55 mm block leaves ~15 mm of the
same tone beyond the outer sensors and ~20 mm in front/behind, so all five
sensors see one surface_value even with a few mm of placement error.

Each block has corner marks and a centre tick in the white gaps (never under a
sensor). Robot faces UP the page: line up a mark on the robot's front (in line
with DN3) with the block's centre tick, and put the DN3 line on the block's
middle.

PRINT: A3, 100 % / actual size, GREYSCALE / BLACK INK ONLY. Check the 100 mm bar.

Change the settings below and rerun:  python3 saturation_sheet.py
"""
from pathlib import Path

PAGE_W, PAGE_H = 420.0, 297.0
COLS, ROWS = 4, 4
BLOCK_W, BLOCK_H = 95.0, 55.0
GAP_X, GAP_Y = 8.0, 11.0
TOP = 16.0

# 8-bit greyscale values, white -> black (denser in the dark range).
VALUES = [255, 224, 192, 160,
          128, 112, 96, 80,
          64, 48, 40, 32,
          24, 16, 8, 0]

TEXT = "#555555"
MARK = "#000000"


def main():
    assert len(VALUES) == COLS * ROWS
    left = (PAGE_W - (COLS * BLOCK_W + (COLS - 1) * GAP_X)) / 2
    parts = []

    for i, v in enumerate(VALUES):
        r, c = divmod(i, COLS)
        x = left + c * (BLOCK_W + GAP_X)
        y = TOP + r * (BLOCK_H + GAP_Y)
        cx = x + BLOCK_W / 2

        # Solid block. White (255) is left unfilled = bare paper, like the course.
        if v < 255:
            parts.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{BLOCK_W}" height="{BLOCK_H}" '
                         f'fill="rgb({v},{v},{v})"/>')

        # Corner marks, 1.5 mm outside the block (in the white gaps).
        o, L = 1.5, 4.0
        for (px, py, dx, dy) in [(x, y, -1, -1), (x + BLOCK_W, y, 1, -1),
                                 (x, y + BLOCK_H, -1, 1), (x + BLOCK_W, y + BLOCK_H, 1, 1)]:
            ox, oy = px + dx * o, py + dy * o
            parts.append(f'<path d="M {ox:.2f} {oy + dy * L:.2f} L {ox:.2f} {oy:.2f} L {ox + dx * L:.2f} {oy:.2f}" '
                         f'fill="none" stroke="{MARK}" stroke-width="0.25"/>')

        # Centre tick above the block (ahead of the robot) + label.
        parts.append(f'<line x1="{cx:.2f}" y1="{y - 1.5:.2f}" x2="{cx:.2f}" y2="{y - 6:.2f}" '
                     f'stroke="{MARK}" stroke-width="0.3"/>')
        pct = round(v / 255 * 100)
        parts.append(f'<text x="{x + 6:.2f}" y="{y - 3:.2f}" font-size="4" fill="{TEXT}">'
                     f'surface_value {v}  ({pct}% white)</text>')

    # Scale check bar and print note.
    yb = PAGE_H - 9.0
    parts.append(f'<line x1="{left:.1f}" y1="{yb}" x2="{left + 100:.1f}" y2="{yb}" stroke="{MARK}" stroke-width="0.4"/>')
    for xx in (left, left + 100):
        parts.append(f'<line x1="{xx:.1f}" y1="{yb - 2}" x2="{xx:.1f}" y2="{yb + 2}" stroke="{MARK}" stroke-width="0.4"/>')
    parts.append(f'<text x="{left + 105:.1f}" y="{yb + 1.2}" font-size="3" fill="{TEXT}">'
                 f'100 mm check bar — print A3 at 100 % (actual size), GREYSCALE / BLACK INK ONLY, '
                 f'same printer and paper as the course. Robot faces up the page.</text>')

    svg = [
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_W:g}mm" height="{PAGE_H:g}mm" '
        f'viewBox="0 0 {PAGE_W:g} {PAGE_H:g}" font-family="Helvetica, Arial, sans-serif">',
        f'<rect x="0" y="0" width="{PAGE_W:g}" height="{PAGE_H:g}" fill="#ffffff"/>',
        *parts,
        '</svg>',
    ]
    out = Path(__file__).with_name("saturation_sheet.svg")
    out.write_text("\n".join(svg) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
