#include "radar/radar_service.h"

#include <cstdio>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "device_state.h"

#define TAG "RadarService"

namespace radar {
namespace {

constexpr uint32_t kTickIntervalMs = 1000;

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

const char* PhaseName(RadarPhase phase) {
    switch (phase) {
        case RadarPhase::kArrived:
            return "arrived";
        case RadarPhase::kNear:
            return "near";
        case RadarPhase::kFar:
            return "far";
        default:
            return "idle";
    }
}

// 交给 LLM 的是「感知描述」而不是成品台词。
// 台词必须由模型按当前人设生成，否则每次到点位说的都一模一样，宠物就死了。
std::string BuildPerception(RadarEvent event, const TreasurePoint& point, double distance_m) {
    const char* name = (point.name != nullptr) ? point.name : "目标";
    char buffer[320];

    switch (event) {
        case RadarEvent::kApproaching:
            std::snprintf(buffer, sizeof(buffer),
                          "（系统感知：%s 就在附近，直线距离约 %.0f 米。这是你身上雷达读到的，"
                          "不是用户在说话。请你用当前角色口吻说一句反应，二十字以内，"
                          "不要念出具体距离数字。）",
                          name, distance_m);
            break;
        case RadarEvent::kArrived:
            std::snprintf(buffer, sizeof(buffer),
                          "（系统感知：已经站到 %s 面前，距离 %.0f 米以内。"
                          "请你用当前角色口吻说一句解锁的话，二十字以内，"
                          "带一点情绪，不要念出具体数字。）",
                          name, distance_m);
            break;
        default:
            std::snprintf(buffer, sizeof(buffer),
                          "（系统感知：%s 已经离开感知范围了。用一句话表示一下情绪，十字以内。）",
                          name);
            break;
    }
    return std::string(buffer);
}

}  // namespace

RadarService& RadarService::GetInstance() {
    static RadarService instance;
    return instance;
}

void RadarService::Start(int uart_num, int rx_pin, int tx_pin, int baud_rate) {
    if (started_) {
        return;
    }
    started_ = true;
    uart_num_ = uart_num;

    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    const uart_port_t port = static_cast<uart_port_t>(uart_num);
    ESP_ERROR_CHECK(uart_driver_install(port, 1024, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    // ATGM336H 只往外吐数据，TX 留着是为了以后能下发配置指令
    ESP_ERROR_CHECK(
        uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(UartTaskEntry, "radar_uart", 4096, this, 5, nullptr);
    xTaskCreate(TickTaskEntry, "radar_tick", 4096, this, 4, nullptr);
    ESP_LOGI(TAG, "started on uart%d (rx=%d tx=%d @%d)", uart_num, rx_pin, tx_pin, baud_rate);
}

void RadarService::SetPoints(const TreasurePoint* points, size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    engine_.SetPoints(points, count);
    ESP_LOGI(TAG, "loaded %u treasure points", static_cast<unsigned>(count));
}

void RadarService::SetConfig(const RadarConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    engine_.SetConfig(config);
}

RadarStatus RadarService::GetStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    const GpsFix fix = has_injected_ ? injected_fix_ : parser_.fix();

    RadarStatus status;
    status.gps_valid = fix.valid;
    status.satellites = fix.satellites;
    status.hdop = fix.hdop;
    status.latitude = fix.latitude;
    status.longitude = fix.longitude;
    status.distance_m = engine_.distance_m();
    status.bearing_deg = engine_.target_bearing_deg();
    status.phase = PhaseName(engine_.phase());

    const TreasurePoint* target = engine_.target();
    if (target != nullptr) {
        status.has_target = true;
        status.target_id = (target->id != nullptr) ? target->id : "";
        status.target_name = (target->name != nullptr) ? target->name : "";
    }
    return status;
}

void RadarService::InjectFix(double latitude, double longitude, int satellites, double hdop) {
    std::lock_guard<std::mutex> lock(mutex_);
    injected_fix_ = GpsFix{};
    injected_fix_.valid = true;
    injected_fix_.latitude = latitude;
    injected_fix_.longitude = longitude;
    injected_fix_.satellites = satellites;
    injected_fix_.hdop = hdop;
    has_injected_ = true;
    ESP_LOGW(TAG, "using injected fix (%.6f, %.6f) - GPS hardware bypassed", latitude, longitude);
}

void RadarService::ClearInjection() {
    std::lock_guard<std::mutex> lock(mutex_);
    has_injected_ = false;
}

void RadarService::UartTaskEntry(void* arg) {
    static_cast<RadarService*>(arg)->UartLoop();
}

void RadarService::TickTaskEntry(void* arg) {
    static_cast<RadarService*>(arg)->TickLoop();
}

void RadarService::UartLoop() {
    uint8_t buffer[256];
    const uart_port_t port = static_cast<uart_port_t>(uart_num_);
    while (true) {
        const int length = uart_read_bytes(port, buffer, sizeof(buffer), pdMS_TO_TICKS(200));
        if (length > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            parser_.Feed(reinterpret_cast<const char*>(buffer), static_cast<size_t>(length));
        }
    }
}

void RadarService::TickLoop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kTickIntervalMs));

        RadarEvent event = RadarEvent::kNone;
        TreasurePoint point = {};
        double distance_m = 0.0;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            const GpsFix fix = has_injected_ ? injected_fix_ : parser_.fix();
            event = engine_.Update(fix, NowMs());
            if (event != RadarEvent::kNone) {
                distance_m = engine_.distance_m();
                const TreasurePoint* target = engine_.target();
                if (target != nullptr) {
                    point = *target;
                }
            }
        }

        if (event != RadarEvent::kNone) {
            HandleEvent(event, &point, distance_m);
        }
    }
}

void RadarService::HandleEvent(RadarEvent event, const TreasurePoint* point,
                               double distance_m) {
    const std::string perception = BuildPerception(event, *point, distance_m);
    ESP_LOGI(TAG, "event=%d distance=%.1fm target=%s", static_cast<int>(event), distance_m,
             (point->name != nullptr) ? point->name : "-");

    auto& app = Application::GetInstance();

    // 本地反馈是确定性的，不依赖网络：先把屏幕点亮 + 提示音，用户知道「有东西」
    if (event == RadarEvent::kArrived) {
        app.Schedule([]() {
            Application::GetInstance().Alert("雷达", "发现了什么…", "happy");
        });
    } else if (event == RadarEvent::kApproaching) {
        app.Schedule([]() {
            Application::GetInstance().Alert("雷达", "信号变强了", "neutral");
        });
    }

    if (announce_mode_ == 1) {
        // 正在说话/正在听用户说话时不要插嘴，留给用户
        if (app.GetDeviceState() == kDeviceStateIdle) {
            Announce(perception);
        } else {
            ESP_LOGI(TAG, "device busy, skip announce");
        }
    }
}

void RadarService::Announce(const std::string& perception) {
    auto& app = Application::GetInstance();
    // WakeWordInvoke 内部发的是 {"type":"listen","state":"detect","text":...}，
    // 相当于「设备自己报告感知到一件事」，服务端会按一轮完整对话处理。
    app.Schedule([perception]() {
        Application::GetInstance().WakeWordInvoke(perception);
    });
}

}  // namespace radar
