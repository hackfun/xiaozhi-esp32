#include "radar/radar_render.h"

#include <cmath>
#include <cstring>

namespace radar {
namespace {

constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
// 拖尾覆盖的角度范围与射线密度。144 条射线铺 80°，最外圈相邻间距约 1px，
// 看不见缝；早期用 56 条时外圈会露出明显的「羽毛」状放射纹。
constexpr int kTailRays = 144;
constexpr float kTailSpanDeg = 80.0f;

inline uint16_t Blend565(uint16_t dst, uint16_t src, uint8_t alpha) {
    if (alpha == 0) {
        return dst;
    }
    if (alpha >= 255) {
        return src;
    }
    const uint32_t inv = 255u - alpha;
    const uint32_t dr = (dst >> 11) & 0x1Fu;
    const uint32_t dg = (dst >> 5) & 0x3Fu;
    const uint32_t db = dst & 0x1Fu;
    const uint32_t sr = (src >> 11) & 0x1Fu;
    const uint32_t sg = (src >> 5) & 0x3Fu;
    const uint32_t sb = src & 0x1Fu;
    const uint32_t r = (dr * inv + sr * alpha) >> 8;
    const uint32_t g = (dg * inv + sg * alpha) >> 8;
    const uint32_t b = (db * inv + sb * alpha) >> 8;
    return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

inline void BlendPixel(uint16_t* buffer, int width, int height, int x, int y, uint16_t color,
                       uint8_t alpha) {
    if (x < 0 || y < 0 || x >= width || y >= height || alpha == 0) {
        return;
    }
    buffer[y * width + x] = Blend565(buffer[y * width + x], color, alpha);
}

void FillCircle(uint16_t* buffer, int width, int height, int cx, int cy, int radius,
                uint16_t color) {
    for (int dy = -radius; dy <= radius; ++dy) {
        const int remaining = radius * radius - dy * dy;
        if (remaining < 0) {
            continue;
        }
        const int span = static_cast<int>(std::sqrt(static_cast<float>(remaining)) + 0.5f);
        const int y = cy + dy;
        if (y < 0 || y >= height) {
            continue;
        }
        int x0 = cx - span;
        int x1 = cx + span;
        if (x0 < 0) x0 = 0;
        if (x1 >= width) x1 = width - 1;
        for (int x = x0; x <= x1; ++x) {
            buffer[y * width + x] = color;
        }
    }
}

void DrawRing(uint16_t* buffer, int width, int height, int cx, int cy, int radius, int thickness,
              uint16_t color, uint8_t alpha) {
    const int outer = radius + thickness / 2;
    const int inner = radius - thickness / 2;
    const int outer2 = outer * outer;
    const int inner2 = inner * inner;
    for (int dy = -outer; dy <= outer; ++dy) {
        const int y = cy + dy;
        if (y < 0 || y >= height) {
            continue;
        }
        for (int dx = -outer; dx <= outer; ++dx) {
            const int d2 = dx * dx + dy * dy;
            if (d2 > outer2 || d2 < inner2) {
                continue;
            }
            BlendPixel(buffer, width, height, cx + dx, y, color, alpha);
        }
    }
}

void DrawLine(uint16_t* buffer, int width, int height, int x0, int y0, int x1, int y1,
              uint16_t color, uint8_t alpha) {
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int error = dx - dy;
    while (true) {
        BlendPixel(buffer, width, height, x0, y0, color, alpha);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int doubled = error * 2;
        if (doubled > -dy) {
            error -= dy;
            x0 += sx;
        }
        if (doubled < dx) {
            error += dx;
            y0 += sy;
        }
    }
}

}  // namespace

void RenderStatic(uint16_t* buffer, int width, int height, const RadarRenderStyle& style) {
    if (buffer == nullptr || width <= 0 || height <= 0) {
        return;
    }

    const int total = width * height;
    for (int i = 0; i < total; ++i) {
        buffer[i] = style.background;
    }

    const int cx = width / 2;
    const int cy = height / 2;
    const int radius = ((width < height) ? width : height) / 2 - 4;

    // 外圈厚一点，像真实仪表的边框
    DrawRing(buffer, width, height, cx, cy, radius, 2, style.grid, 235);
    // 两档内环：1/3 与 2/3 量程
    DrawRing(buffer, width, height, cx, cy, radius * 2 / 3, 1, style.grid, 175);
    DrawRing(buffer, width, height, cx, cy, radius / 3, 1, style.grid, 175);

    // 十字线
    DrawLine(buffer, width, height, cx - radius + 6, cy, cx + radius - 6, cy, style.grid, 95);
    DrawLine(buffer, width, height, cx, cy - radius + 6, cx, cy + radius - 6, style.grid, 95);

    // 外圈方位刻度，每 30 度一格；12 点方向加长一点
    for (int i = 0; i < 12; ++i) {
        const float angle = (i * 30.0f) * kDegToRad;
        const float dx = std::sin(angle);
        const float dy = -std::cos(angle);
        const int length = (i % 3 == 0) ? 10 : 5;
        const int x_outer = cx + static_cast<int>(dx * (radius - 2) + 0.5f);
        const int y_outer = cy + static_cast<int>(dy * (radius - 2) + 0.5f);
        const int x_inner = cx + static_cast<int>(dx * (radius - 2 - length) + 0.5f);
        const int y_inner = cy + static_cast<int>(dy * (radius - 2 - length) + 0.5f);
        DrawLine(buffer, width, height, x_inner, y_inner, x_outer, y_outer, style.sweep, 190);
    }
}

void RenderFrame(uint16_t* buffer, int width, int height, const uint16_t* static_layer,
                 const RadarRenderInput& input, const RadarRenderStyle& style) {
    if (buffer == nullptr || width <= 0 || height <= 0) {
        return;
    }

    if (static_layer != nullptr) {
        std::memcpy(buffer, static_layer, static_cast<size_t>(width) * height * sizeof(uint16_t));
    } else {
        RenderStatic(buffer, width, height, style);
    }

    const int cx = width / 2;
    const int cy = height / 2;
    const int radius = ((width < height) ? width : height) / 2 - 4;

    // --- 扫描扇形：从头部往回铺 kTailRays 条射线，透明度平方衰减 ---
    const float head = input.sweep_angle_deg * kDegToRad;
    for (int i = 0; i < kTailRays; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(kTailRays);
        const float angle = head - t * kTailSpanDeg * kDegToRad;
        const float dx = std::sin(angle);
        const float dy = -std::cos(angle);
        // 线性衰减：平方衰减会让中段塌掉，扇形看着只剩一条细线
        const uint8_t alpha = static_cast<uint8_t>(205.0f * (1.0f - t));
        if (alpha == 0) {
            continue;
        }
        for (int r = 6; r < radius; ++r) {
            BlendPixel(buffer, width, height, cx + static_cast<int>(dx * r + 0.5f),
                       cy + static_cast<int>(dy * r + 0.5f), style.sweep, alpha);
        }
    }

    // 扫描头单独一条亮线
    {
        const float dx = std::sin(head);
        const float dy = -std::cos(head);
        for (int r = 6; r < radius; ++r) {
            BlendPixel(buffer, width, height, cx + static_cast<int>(dx * r + 0.5f),
                       cy + static_cast<int>(dy * r + 0.5f), style.sweep, 235);
        }
    }

    // --- 信号点：没有方位信息，固定在 12 点方向，半径表示距离 ---
    if (input.distance_m >= 0.0f && input.max_range_m > 0.0f) {
        float ratio = input.distance_m / input.max_range_m;
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        const int signal_radius = 8 + static_cast<int>(ratio * (radius - 16));
        const int sx = cx;
        const int sy = cy - signal_radius;

        const bool alarming = input.proximity > 0.75f;
        const uint16_t color = alarming ? style.alarm : style.signal;

        // 脉冲圈：phase 从 0 到 1 时扩散并淡出
        const int pulse_radius = 4 + static_cast<int>(input.pulse_phase * 11.0f);
        const uint8_t pulse_alpha =
            static_cast<uint8_t>((1.0f - input.pulse_phase) * (alarming ? 230.0f : 180.0f));
        DrawRing(buffer, width, height, sx, sy, pulse_radius, 1, color, pulse_alpha);
        FillCircle(buffer, width, height, sx, sy, 3, color);
    }

    // --- 外圈进度：从 12 点顺时针点亮，长度表示接近程度 ---
    if (input.proximity > 0.0f) {
        const int segments = 60;
        const int lit = static_cast<int>(input.proximity * segments + 0.5f);
        for (int i = 0; i < lit; ++i) {
            const float angle = (static_cast<float>(i) / segments) * 360.0f * kDegToRad;
            const float dx = std::sin(angle);
            const float dy = -std::cos(angle);
            const int x = cx + static_cast<int>(dx * (radius + 1) + 0.5f);
            const int y = cy + static_cast<int>(dy * (radius + 1) + 0.5f);
            BlendPixel(buffer, width, height, x, y, style.sweep, 220);
        }
    }
}

}  // namespace radar
