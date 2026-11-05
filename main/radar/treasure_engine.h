#ifndef RADAR_TREASURE_ENGINE_H
#define RADAR_TREASURE_ENGINE_H

// 寻宝状态机：把抖动的 GPS 距离，变成稳定的「远 / 接近 / 到达」三态与事件。
// 三个关键点，都是实测踩出来的：
//   1. 中位数滤波——民用 GPS 定位是跳变的，原始距离直接判会抽风
//   2. 迟滞半径——进入用 15m、离开用 30m，避免站在边界上反复触发
//   3. 确认时长——要连续满足数秒才认，单点毛刺不算数
// 不依赖 ESP-IDF，可以在 PC 上单元测试。

#include <cstddef>
#include <cstdint>

#include "radar/geo_utils.h"
#include "radar/nmea_parser.h"

namespace radar {

enum class RadarEvent {
    kNone,
    kApproaching,  // 首次进入接近半径
    kArrived,      // 首次进入到达半径，可解锁
    kLeft,         // 离开点位
};

enum class RadarPhase { kIdle, kFar, kNear, kArrived };

struct TreasurePoint {
    const char* id = nullptr;
    const char* name = nullptr;
    GeoPoint position;
};

struct RadarConfig {
    double arrive_radius_m = 5.0;     // 解锁半径
    double approach_radius_m = 15.0;  // 开始提示的半径
    double leave_radius_m = 30.0;     // 迟滞：走出去这么远才算离开
    int min_satellites = 5;
    double max_hdop = 3.5;
    uint32_t confirm_ms = 3000;    // 连续满足多久才认
    uint32_t cooldown_ms = 90000;  // 同一事件重复播报的最小间隔
};

class TreasureEngine {
public:
    static constexpr size_t kWindowSize = 5;
    static constexpr size_t kNoTarget = static_cast<size_t>(-1);

    void SetConfig(const RadarConfig& config) { config_ = config; }
    const RadarConfig& config() const { return config_; }

    void SetPoints(const TreasurePoint* points, size_t count);
    size_t point_count() const { return point_count_; }

    // 建议 1Hz 调用。返回本帧产生的事件（最多一个）。
    RadarEvent Update(const GpsFix& fix, uint32_t now_ms);

    RadarPhase phase() const { return phase_; }
    double distance_m() const { return smoothed_distance_m_; }
    const TreasurePoint* target() const {
        return target_index_ == kNoTarget ? nullptr : &points_[target_index_];
    }
    // 距目标的方位角（真北），没有目标时返回 0
    double target_bearing_deg() const { return target_bearing_deg_; }

private:
    double PushDistance(double value);
    void ResetWindow();
    // 冷却按事件类型分别计，否则「到达」的冷却会把随后的「离开」一起吞掉
    bool ShouldEmit(RadarEvent event, uint32_t now_ms);

    RadarConfig config_;
    TreasurePoint points_[32] = {};
    size_t point_count_ = 0;

    size_t target_index_ = kNoTarget;
    RadarPhase phase_ = RadarPhase::kIdle;
    RadarPhase candidate_ = RadarPhase::kIdle;
    uint32_t candidate_since_ms_ = 0;
    uint32_t last_valid_ms_ = 0;
    uint32_t last_emit_approaching_ms_ = 0;
    uint32_t last_emit_arrived_ms_ = 0;
    uint32_t last_emit_left_ms_ = 0;
    double smoothed_distance_m_ = 0.0;
    double target_bearing_deg_ = 0.0;

    double window_[kWindowSize] = {};
    size_t window_index_ = 0;
    size_t window_count_ = 0;
};

}  // namespace radar

#endif  // RADAR_TREASURE_ENGINE_H
