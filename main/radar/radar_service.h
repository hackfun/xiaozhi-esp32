#ifndef RADAR_SERVICE_H
#define RADAR_SERVICE_H

// 把「串口 GPS 流 -> 状态机事件 -> AI 开口」串起来。
// 依赖 ESP-IDF 与固件 Application，因此不能参与 host 端测试；
// 纯算法部分在 geo_utils / nmea_parser / treasure_engine 三个文件里。

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "radar/nmea_parser.h"
#include "radar/treasure_engine.h"

namespace radar {

struct RadarStatus {
    bool gps_valid = false;
    int satellites = 0;
    double hdop = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    bool has_target = false;
    std::string target_id;
    std::string target_name;
    double distance_m = 0.0;
    double bearing_deg = 0.0;
    const char* phase = "idle";
};

class RadarService {
public:
    static RadarService& GetInstance();

    // 启动串口读取与状态机轮询。重复调用是安全的。
    void Start(int uart_num, int rx_pin, int tx_pin, int baud_rate);
    void SetPoints(const TreasurePoint* points, size_t count);
    void SetConfig(const RadarConfig& config);

    RadarStatus GetStatus();

    // 没有 GPS 硬件时手动灌一条定位，先把「感知 -> 播报」链路跑通
    void InjectFix(double latitude, double longitude, int satellites = 9, double hdop = 0.85);
    void ClearInjection();

    // 0 = 只做本地反馈（屏幕/提示音），1 = 同时让 AI 开口
    void SetAnnounceMode(int mode) { announce_mode_ = mode; }
    int announce_mode() const { return announce_mode_; }

private:
    RadarService() = default;

    static void UartTaskEntry(void* arg);
    static void TickTaskEntry(void* arg);
    void UartLoop();
    void TickLoop();
    void HandleEvent(RadarEvent event, const TreasurePoint* point, double distance_m);
    void Announce(const std::string& perception);

    std::mutex mutex_;
    NmeaParser parser_;
    TreasureEngine engine_;

    GpsFix injected_fix_;
    bool has_injected_ = false;

    bool started_ = false;
    int uart_num_ = -1;
    int announce_mode_ = 1;
};

}  // namespace radar

#endif  // RADAR_SERVICE_H
