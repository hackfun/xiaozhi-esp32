#include "radar/radar_link.h"

#include <cstdio>
#include <cstring>
#include <string>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "application.h"
#include "device_state.h"

#define TAG "RadarLink"

namespace radar {
namespace {

uint32_t NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// 交给 LLM 的是「感知描述」，台词由模型按人设生成。
// 写死文案的话每次到点位说的都一样，宠物就死了。
std::string BuildPerception(const radar_packet_t& packet) {
    char name[25] = {};
    std::memcpy(name, packet.point_name, sizeof(packet.point_name));
    name[24] = '\0';
    if (name[0] == '\0') {
        std::snprintf(name, sizeof(name), "目标");
    }

    const float meters = static_cast<float>(packet.distance_cm) / 100.0f;
    char text[320];

    switch (packet.phase) {
        case kPhaseNear:
            std::snprintf(text, sizeof(text),
                          "（系统感知：手表报告目标「%s」在 %.0f 米内，已进入接近范围。"
                          "这是手表读到的，不是用户在说话。用当前角色口吻说一句反应，"
                          "二十字以内，不要念出具体数字。）",
                          name, meters);
            break;
        case kPhaseArrived:
            std::snprintf(text, sizeof(text),
                          "（系统感知：已经走到「%s」跟前了，%.0f 米以内。"
                          "用当前角色口吻说一句解锁的话，二十字以内，带一点情绪，"
                          "不要念出具体数字。）",
                          name, meters);
            break;
        case kPhaseFar:
            std::snprintf(text, sizeof(text),
                          "（系统感知：目标「%s」已经离开感知范围。用一句话表示一下情绪，"
                          "十字以内。）",
                          name);
            break;
        default:
            std::snprintf(text, sizeof(text), "（系统感知：手表上有新状态，但信息不完整。）");
            break;
    }
    return std::string(text);
}

}  // namespace

RadarLink& RadarLink::GetInstance() {
    static RadarLink instance;
    return instance;
}

void RadarLink::Start() {
    if (watching_) {
        return;
    }
    watching_ = true;
    xTaskCreate(WatchTaskEntry, "radar_link", 4096, this, 3, nullptr);
    ESP_LOGI(TAG, "watcher started");
}

void RadarLink::WatchTaskEntry(void* arg) {
    static_cast<RadarLink*>(arg)->WatchLoop();
}

void RadarLink::WatchLoop() {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (running_) {
            continue;
        }
        // ESP-NOW 必须在 WiFi 起来之后初始化，否则信道是默认值，收不到手表
        wifi_ap_record_t ap = {};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            if (DoInit()) {
                ESP_LOGI(TAG, "ready on channel %u (AP: %s)", ap.primary,
                         reinterpret_cast<const char*>(ap.ssid));
                // 等设备把开机提示音和状态都刷完，否则 Alert 会被盖掉
                vTaskDelay(pdMS_TO_TICKS(4000));
                ShowChannelOnScreen(ap.primary);
            }
        }
    }
}

void RadarLink::ShowChannelOnScreen(uint8_t channel) {
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "信道 %u", channel);
    const std::string text(buffer);
    auto& app = Application::GetInstance();
    app.Schedule([text]() {
        Application::GetInstance().Alert("雷达已就绪", text.c_str(), "neutral");
    });
}

bool RadarLink::DoInit() {
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_now_register_recv_cb(&RadarLink::OnReceive);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register recv cb failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        return false;
    }

    esp_now_peer_info_t peer = {};
    std::memcpy(peer.peer_addr, broadcast_mac_, 6);
    peer.channel = 0;  // 0 = 跟随当前信道
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(peer.peer_addr)) {
        err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add broadcast peer failed: %s", esp_err_to_name(err));
            esp_now_deinit();
            return false;
        }
    }

    running_ = true;
    return true;
}

RadarLink::Snapshot RadarLink::GetSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

void RadarLink::OnReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
    if (data == nullptr || length < static_cast<int>(sizeof(radar_packet_t))) {
        return;
    }
    radar_packet_t packet;
    std::memcpy(&packet, data, sizeof(packet));
    if (packet.magic != RADAR_PROTOCOL_MAGIC) {
        return;
    }
    const int8_t rssi = (info != nullptr && info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : 0;
    GetInstance().HandlePacket(packet, rssi);
}

void RadarLink::HandlePacket(const radar_packet_t& packet, int8_t rssi) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.valid = true;
        snapshot_.phase = packet.phase;
        snapshot_.distance_cm = packet.distance_cm;
        snapshot_.satellites = packet.satellites;
        snapshot_.rssi = rssi;
        snapshot_.updated_ms = NowMs();
        std::memcpy(snapshot_.point_name, packet.point_name, sizeof(packet.point_name));
        snapshot_.point_name[24] = '\0';
    }

    if (packet.type == kRadarPacketPing) {
        // 手表在找我，回一个包让它测 RSSI
        ReplyToPing();
        return;
    }
    if (packet.type != kRadarPacketEvent) {
        return;
    }
    ESP_LOGI(TAG, "event phase=%u distance=%.1fm rssi=%d", packet.phase,
             packet.distance_cm / 100.0f, rssi);
    Announce(packet);
}

void RadarLink::ReplyToPing() {
    radar_packet_t reply = {};
    reply.magic = RADAR_PROTOCOL_MAGIC;
    reply.version = RADAR_PROTOCOL_VERSION;
    reply.type = kRadarPacketPing;
    const esp_err_t err =
        esp_now_send(broadcast_mac_, reinterpret_cast<const uint8_t*>(&reply), sizeof(reply));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ping reply failed: %s", esp_err_to_name(err));
    }
}

void RadarLink::Announce(const radar_packet_t& packet) {
    auto& app = Application::GetInstance();
    // 正在说话或正在听用户说话时不插嘴——用户优先
    if (app.GetDeviceState() != kDeviceStateIdle) {
        ESP_LOGI(TAG, "device busy, skip announce");
        return;
    }
    const std::string perception = BuildPerception(packet);
    app.Schedule([perception]() { Application::GetInstance().WakeWordInvoke(perception); });
}

}  // namespace radar
