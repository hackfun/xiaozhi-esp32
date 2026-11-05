#ifndef RADAR_RENDER_H
#define RADAR_RENDER_H

// 雷达盘面的纯绘制层：给定一块 RGB565 buffer，画出一帧。
// 刻意不依赖 LVGL 与 ESP-IDF——一是 LVGL 8/9 绘图 API 差异大，二是这样能在 PC 上复刻验证。
// 文字不在这里画（避免引入字体依赖），距离读数由上层用 label 叠上去。

#include <cstdint>

namespace radar {

// RGB565 常量
constexpr uint16_t kColorBackground = 0x00A1;  // #05140F 深绿黑
constexpr uint16_t kColorGrid = 0x1CEE;        // #1D9E75 暗绿（比 #0F6E56 亮一档，否则环看不清）
constexpr uint16_t kColorGridDim = 0x0B6A;     // 更暗的环
constexpr uint16_t kColorSweep = 0x3FF3;       // #3DFF9E 亮绿
constexpr uint16_t kColorSignal = 0x07EC;      // #00E676 信号绿
constexpr uint16_t kColorAlarm = 0xFB00;       // #FF6B00 近距警示橙

struct RadarRenderStyle {
    uint16_t background = kColorBackground;
    uint16_t grid = kColorGrid;
    uint16_t sweep = kColorSweep;
    uint16_t signal = kColorSignal;
    uint16_t alarm = kColorAlarm;
};

struct RadarRenderInput {
    int width = 240;
    int height = 240;
    // 扫描线当前角度（度），0 = 正上方，顺时针增加
    float sweep_angle_deg = 0.0f;
    // 目标距离（米）。负值表示没有目标。
    float distance_m = -1.0f;
    // 盘面最大量程（米），对应最外圈
    float max_range_m = 30.0f;
    // 接近程度 0~1，用于决定颜色和脉冲强度
    float proximity = 0.0f;
    // 脉冲相位 0~1，由调用方按时间推进
    float pulse_phase = 0.0f;
};

// 静态层：背景圆、三档同心环、十字线、外圈刻度。只画一次，之后每帧 memcpy 复用。
void RenderStatic(uint16_t* buffer, int width, int height, const RadarRenderStyle& style);

// 动态层：在静态层基础上叠加扫描扇形、拖尾、信号点、强度弧。
// static_layer 可为 nullptr（此时会先自己画一遍静态层，用于调试）。
void RenderFrame(uint16_t* buffer, int width, int height, const uint16_t* static_layer,
                 const RadarRenderInput& input, const RadarRenderStyle& style);

}  // namespace radar

#endif  // RADAR_RENDER_H
