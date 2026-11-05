"""把 radar_render.cc 的算法原样复刻到 Python，输出 BMP 预览。

用途：改雷达视觉前先在这里看效果，不用烧板子。算法改动时两边要同步，
否则预览会和真机对不上。

    python tests/radar/preview.py

产出 tests/radar/preview.bmp（480x480，四种状态拼成 2x2）。
"""

import math
import struct

W = H = 240

K_BACKGROUND = 0x00A1
K_GRID = 0x1CEE
K_SWEEP = 0x3FF3
K_SIGNAL = 0x07EC
K_ALARM = 0xFB00

TAIL_RAYS = 144
TAIL_SPAN_DEG = 80.0


def blend565(dst, src, alpha):
    if alpha == 0:
        return dst
    if alpha >= 255:
        return src
    inv = 255 - alpha
    dr, dg, db = (dst >> 11) & 0x1F, (dst >> 5) & 0x3F, dst & 0x1F
    sr, sg, sb = (src >> 11) & 0x1F, (src >> 5) & 0x3F, src & 0x1F
    r = (dr * inv + sr * alpha) >> 8
    g = (dg * inv + sg * alpha) >> 8
    b = (db * inv + sb * alpha) >> 8
    return (r << 11) | (g << 5) | b


class Canvas:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.px = [K_BACKGROUND] * (w * h)

    def blend(self, x, y, color, alpha):
        if x < 0 or y < 0 or x >= self.w or y >= self.h or alpha == 0:
            return
        i = y * self.w + x
        self.px[i] = blend565(self.px[i], color, alpha)

    def fill_circle(self, cx, cy, radius, color):
        for dy in range(-radius, radius + 1):
            remain = radius * radius - dy * dy
            if remain < 0:
                continue
            span = int(math.sqrt(remain) + 0.5)
            y = cy + dy
            if y < 0 or y >= self.h:
                continue
            for x in range(max(0, cx - span), min(self.w - 1, cx + span) + 1):
                self.px[y * self.w + x] = color

    def ring(self, cx, cy, radius, thickness, color, alpha):
        outer = radius + thickness // 2
        inner = radius - thickness // 2
        o2, i2 = outer * outer, inner * inner
        for dy in range(-outer, outer + 1):
            y = cy + dy
            if y < 0 or y >= self.h:
                continue
            for dx in range(-outer, outer + 1):
                d2 = dx * dx + dy * dy
                if d2 > o2 or d2 < i2:
                    continue
                self.blend(cx + dx, y, color, alpha)

    def line(self, x0, y0, x1, y1, color, alpha):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            self.blend(x0, y0, color, alpha)
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy


def render_static(c):
    c.px = [K_BACKGROUND] * (c.w * c.h)
    cx, cy = c.w // 2, c.h // 2
    radius = min(c.w, c.h) // 2 - 4
    c.ring(cx, cy, radius, 2, K_GRID, 235)
    c.ring(cx, cy, radius * 2 // 3, 1, K_GRID, 175)
    c.ring(cx, cy, radius // 3, 1, K_GRID, 175)
    c.line(cx - radius + 6, cy, cx + radius - 6, cy, K_GRID, 95)
    c.line(cx, cy - radius + 6, cx, cy + radius - 6, K_GRID, 95)
    for i in range(12):
        a = math.radians(i * 30.0)
        dx, dy = math.sin(a), -math.cos(a)
        length = 10 if i % 3 == 0 else 5
        c.line(cx + int(dx * (radius - 2 - length) + 0.5),
               cy + int(dy * (radius - 2 - length) + 0.5),
               cx + int(dx * (radius - 2) + 0.5),
               cy + int(dy * (radius - 2) + 0.5), K_SWEEP, 190)
    return list(c.px)


def render_frame(static, sweep_deg, distance_m, proximity, pulse, max_range=30.0):
    c = Canvas(W, H)
    c.px = list(static)
    cx, cy = W // 2, H // 2
    radius = min(W, H) // 2 - 4

    head = math.radians(sweep_deg)
    for i in range(TAIL_RAYS):
        t = i / TAIL_RAYS
        a = head - t * math.radians(TAIL_SPAN_DEG)
        dx, dy = math.sin(a), -math.cos(a)
        alpha = int(205.0 * (1.0 - t))
        if alpha == 0:
            continue
        for r in range(6, radius):
            c.blend(cx + int(dx * r + 0.5), cy + int(dy * r + 0.5), K_SWEEP, alpha)

    dx, dy = math.sin(head), -math.cos(head)
    for r in range(6, radius):
        c.blend(cx + int(dx * r + 0.5), cy + int(dy * r + 0.5), K_SWEEP, 235)

    if distance_m >= 0:
        ratio = min(distance_m / max_range, 1.0)
        sr = 8 + int(ratio * (radius - 16))
        sx, sy = cx, cy - sr
        alarming = proximity > 0.75
        color = K_ALARM if alarming else K_SIGNAL
        pr = 4 + int(pulse * 11.0)
        pa = int((1.0 - pulse) * (230.0 if alarming else 180.0))
        c.ring(sx, sy, pr, 1, color, pa)
        c.fill_circle(sx, sy, 3, color)

    if proximity > 0:
        segments = 60
        lit = int(proximity * segments + 0.5)
        for i in range(lit):
            a = math.radians(i / segments * 360.0)
            c.blend(cx + int(math.sin(a) * (radius + 1) + 0.5),
                    cy + int(-math.cos(a) * (radius + 1) + 0.5), K_SWEEP, 220)

    return c.px


def write_bmp(path, pixels, w, h):
    row_size = (w * 3 + 3) & ~3
    data = bytearray()
    for y in range(h - 1, -1, -1):
        row = bytearray()
        for x in range(w):
            c = pixels[y * w + x]
            r5, g6, b5 = (c >> 11) & 0x1F, (c >> 5) & 0x3F, c & 0x1F
            row += bytes(((b5 << 3) | (b5 >> 2), (g6 << 2) | (g6 >> 4), (r5 << 3) | (r5 >> 2)))
        row += b"\x00" * (row_size - w * 3)
        data += row

    size = len(data)
    with open(path, "wb") as f:
        f.write(b"BM")
        f.write(struct.pack("<IHHI", 14 + 40 + size, 0, 0, 54))
        f.write(struct.pack("<IiiHHIIiiII", 40, w, h, 1, 24, 0, size, 2835, 2835, 0, 0))
        f.write(data)


def main():
    static = render_static(Canvas(W, H))
    cases = [
        ("无目标", 20.0, -1.0, 0.0, 0.3),
        ("远处 25m", 95.0, 25.0, 0.17, 0.6),
        ("接近 12m", 160.0, 12.0, 0.60, 0.4),
        ("到达 3m", 250.0, 3.0, 0.90, 0.2),
    ]

    out_w, out_h = W * 2, H * 2
    combined = [K_BACKGROUND] * (out_w * out_h)
    for idx, (_, sweep, dist, prox, pulse) in enumerate(cases):
        frame = render_frame(static, sweep, dist, prox, pulse)
        ox, oy = (idx % 2) * W, (idx // 2) * H
        for y in range(H):
            for x in range(W):
                combined[(oy + y) * out_w + ox + x] = frame[y * W + x]
        print(f"  {idx + 1}. {cases[idx][0]}: sweep={sweep} dist={dist} prox={prox}")

    path = "tests/radar/preview.bmp"
    write_bmp(path, combined, out_w, out_h)
    print(f"\n已输出 {path} ({out_w}x{out_h})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
