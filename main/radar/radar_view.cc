#include "radar/radar_view.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>

#include "display/display.h"
#include "radar/radar_service.h"

#define TAG "RadarView"

namespace radar {
namespace {

// 圆屏固定 240x240。换屏时这里和 config.h 一起改。
constexpr int kCanvasWidth = 240;
constexpr int kCanvasHeight = 240;
// 66ms ≈ 15fps。240x240 RGB565 一帧 115KB，40MHz SPI 传完要 23ms，
// 再快就开始和音频/网络抢带宽了。
constexpr uint32_t kFrameIntervalMs = 66;
constexpr float kSweepDegPerFrame = 360.0f / (3200.0f / kFrameIntervalMs);  // 3.2 秒一圈

}  // namespace

RadarView& RadarView::GetInstance() {
    static RadarView instance;
    return instance;
}

void RadarView::Attach(Display* display) {
    if (attached_ || display == nullptr) {
        return;
    }
    display_ = display;
    attached_ = true;
    xTaskCreate(TaskEntry, "radar_view", 6144, this, 3, nullptr);
    ESP_LOGI(TAG, "attached");
}

void RadarView::SetVisible(bool visible) {
    if (visible_ == visible) {
        return;
    }
    visible_ = visible;
    if (display_ == nullptr) {
        return;
    }
    DisplayLockGuard lock(display_);
    if (created_ && canvas_ != nullptr) {
        if (visible) {
            lv_obj_remove_flag(static_cast<lv_obj_t*>(canvas_), LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(static_cast<lv_obj_t*>(canvas_));
        } else {
            lv_obj_add_flag(static_cast<lv_obj_t*>(canvas_), LV_OBJ_FLAG_HIDDEN);
        }
    }
}

bool RadarView::EnsureCreated() {
    if (created_) {
        return true;
    }

    frame_buf_ = static_cast<uint16_t*>(
        heap_caps_malloc(kCanvasWidth * kCanvasHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    static_buf_ = static_cast<uint16_t*>(
        heap_caps_malloc(kCanvasWidth * kCanvasHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    if (frame_buf_ == nullptr || static_buf_ == nullptr) {
        ESP_LOGE(TAG, "frame buffer alloc failed (need %d KB PSRAM)",
                 static_cast<int>(2 * kCanvasWidth * kCanvasHeight * sizeof(uint16_t) / 1024));
        return false;
    }

    RenderStatic(static_buf_, kCanvasWidth, kCanvasHeight, RadarRenderStyle{});

    lv_obj_t* canvas = lv_canvas_create(lv_screen_active());
    if (canvas == nullptr) {
        ESP_LOGE(TAG, "canvas create failed");
        return false;
    }
    lv_canvas_set_buffer(canvas, frame_buf_, kCanvasWidth, kCanvasHeight, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(canvas, 0, 0);
    lv_obj_remove_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(canvas);
    if (!visible_) {
        lv_obj_add_flag(canvas, LV_OBJ_FLAG_HIDDEN);
    }

    canvas_ = canvas;
    created_ = true;
    ESP_LOGI(TAG, "canvas ready");
    return true;
}

void RadarView::Destroy() {
    if (canvas_ != nullptr) {
        lv_obj_delete(static_cast<lv_obj_t*>(canvas_));
        canvas_ = nullptr;
    }
    if (frame_buf_ != nullptr) {
        heap_caps_free(frame_buf_);
        frame_buf_ = nullptr;
    }
    if (static_buf_ != nullptr) {
        heap_caps_free(static_buf_);
        static_buf_ = nullptr;
    }
    created_ = false;
}

void RadarView::TaskEntry(void* arg) {
    static_cast<RadarView*>(arg)->Loop();
}

void RadarView::Loop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kFrameIntervalMs));
        if (display_ == nullptr) {
            continue;
        }
        // SetupUI 还没跑完时 screen 上什么都没有，这时候建 canvas 会被后续 UI 盖掉
        if (!display_->IsSetupUICalled()) {
            continue;
        }

        const RadarStatus status = RadarService::GetInstance().GetStatus();

        // 只在「有这个点位、且在量程内」时才让雷达占屏，平时把屏幕还给对话界面
        const bool should_show =
            status.has_target && status.gps_valid && status.distance_m <= 30.0;
        if (should_show != visible_) {
            SetVisible(should_show);
        }
        if (!visible_) {
            continue;
        }

        sweep_deg_ += kSweepDegPerFrame;
        if (sweep_deg_ >= 360.0f) {
            sweep_deg_ -= 360.0f;
        }

        DisplayLockGuard lock(display_);
        if (!EnsureCreated()) {
            continue;
        }

        RadarRenderInput input;
        input.width = kCanvasWidth;
        input.height = kCanvasHeight;
        input.sweep_angle_deg = sweep_deg_;
        input.distance_m = status.has_target ? static_cast<float>(status.distance_m) : -1.0f;
        input.max_range_m = 30.0f;
        // 30m 外算 0，5m 内算 1
        float proximity = 0.0f;
        if (status.has_target) {
            const float d = static_cast<float>(status.distance_m);
            proximity = (30.0f - d) / 25.0f;
            if (proximity < 0.0f) proximity = 0.0f;
            if (proximity > 1.0f) proximity = 1.0f;
        }
        input.proximity = proximity;
        // 越近脉冲越快：1.6s 到 0.6s
        const float pulse_period_ms = 1600.0f - proximity * 1000.0f;
        input.pulse_phase = static_cast<float>(esp_timer_get_time() / 1000.0) / pulse_period_ms;
        input.pulse_phase -= static_cast<int>(input.pulse_phase);

        RenderFrame(frame_buf_, kCanvasWidth, kCanvasHeight, static_buf_, input,
                    RadarRenderStyle{});
        lv_obj_invalidate(static_cast<lv_obj_t*>(canvas_));
    }
}

}  // namespace radar
