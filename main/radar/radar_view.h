#ifndef RADAR_VIEW_H
#define RADAR_VIEW_H

// 把 radar_render 画出来的帧推到 LVGL canvas 上，并驱动扫描动画。
// 单独一个 20fps 的任务：状态机是 1Hz 的，动画不能跟着 1Hz 走。

#include <cstdint>

#include "radar/radar_render.h"

class Display;

namespace radar {

class RadarView {
public:
    static RadarView& GetInstance();

    // 绑定显示设备并起动画任务。重复调用是安全的。
    void Attach(Display* display);
    void SetVisible(bool visible);

private:
    RadarView() = default;
    static void TaskEntry(void* arg);
    void Loop();
    bool EnsureCreated();
    void Destroy();

    Display* display_ = nullptr;
    void* canvas_ = nullptr;  // lv_obj_t*
    uint16_t* frame_buf_ = nullptr;
    uint16_t* static_buf_ = nullptr;
    bool created_ = false;
    bool visible_ = true;
    bool attached_ = false;
    float sweep_deg_ = 0.0f;
};

}  // namespace radar

#endif  // RADAR_VIEW_H
