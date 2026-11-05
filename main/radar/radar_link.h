#ifndef RADAR_LINK_H
#define RADAR_LINK_H

// 掌机侧的雷达接收端：接手表上报的寻宝事件，转成一圈对话让 AI 开口。
//
// 掌机的 WiFi 已经在连服务器，这里只做 esp_now_init + 注册回调，
// 不会重新初始化 WiFi——重新 init 会直接把现有连接打断。
// ESP-NOW 在 STA 模式下自动跟随 AP 信道，所以手表必须配置成同一个信道。

#include <cstdint>
#include <mutex>

#include <esp_now.h>

#include "radar/radar_protocol.h"

namespace radar {

class RadarLink {
public:
    static RadarLink& GetInstance();

    // 随时可调。内部会等 WiFi 连上再真正初始化 ESP-NOW。
    void Start();
    bool running() const { return running_; }

    struct Snapshot {
        bool valid = false;
        uint8_t phase = kPhaseIdle;
        uint16_t distance_cm = 0;
        uint8_t satellites = 0;
        int8_t rssi = 0;          // 手表侧信号强度，可用来判断手表远近
        uint32_t updated_ms = 0;
        char point_name[25] = {};
    };
    Snapshot GetSnapshot();

private:
    RadarLink() = default;

    static void WatchTaskEntry(void* arg);
    static void OnReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length);

    void WatchLoop();
    bool DoInit();
    void HandlePacket(const radar_packet_t& packet, int8_t rssi);
    void ReplyToPing();
    void Announce(const radar_packet_t& packet);

    bool watching_ = false;
    bool running_ = false;
    mutable std::mutex mutex_;
    Snapshot snapshot_;
    uint8_t broadcast_mac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

}  // namespace radar

#endif  // RADAR_LINK_H
