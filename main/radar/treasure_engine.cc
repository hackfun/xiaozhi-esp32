#include "radar/treasure_engine.h"

#include <cmath>

namespace radar {
namespace {
// 超过这么久没有可用定位，就判定为「跟丢」，回 idle，避免拿着旧坐标瞎报
constexpr uint32_t kFixTimeoutMs = 60000;
}  // namespace

void TreasureEngine::SetPoints(const TreasurePoint* points, size_t count) {
    if (points == nullptr) {
        point_count_ = 0;
        target_index_ = kNoTarget;
        phase_ = RadarPhase::kIdle;
        candidate_ = RadarPhase::kIdle;
        return;
    }
    point_count_ = (count > 32) ? 32 : count;
    for (size_t i = 0; i < point_count_; ++i) {
        points_[i] = points[i];
    }
    target_index_ = kNoTarget;
    phase_ = RadarPhase::kIdle;
    candidate_ = RadarPhase::kIdle;
    last_emit_approaching_ms_ = 0;
    last_emit_arrived_ms_ = 0;
    last_emit_left_ms_ = 0;
    ResetWindow();
}

void TreasureEngine::ResetWindow() {
    window_index_ = 0;
    window_count_ = 0;
    smoothed_distance_m_ = 0.0;
}

double TreasureEngine::PushDistance(double value) {
    window_[window_index_] = value;
    window_index_ = (window_index_ + 1) % kWindowSize;
    if (window_count_ < kWindowSize) {
        ++window_count_;
    }

    // 元素最多 5 个，插入排序最省事
    double sorted[kWindowSize];
    for (size_t i = 0; i < window_count_; ++i) {
        sorted[i] = window_[i];
    }
    for (size_t i = 1; i < window_count_; ++i) {
        const double key = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > key) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = key;
    }
    return sorted[window_count_ / 2];
}

bool TreasureEngine::ShouldEmit(RadarEvent event, uint32_t now_ms) {
    uint32_t* last = nullptr;
    switch (event) {
        case RadarEvent::kApproaching:
            last = &last_emit_approaching_ms_;
            break;
        case RadarEvent::kArrived:
            last = &last_emit_arrived_ms_;
            break;
        case RadarEvent::kLeft:
            last = &last_emit_left_ms_;
            break;
        default:
            return false;
    }
    if (*last != 0 && (now_ms - *last) < config_.cooldown_ms) {
        return false;
    }
    *last = now_ms;
    return true;
}

RadarEvent TreasureEngine::Update(const GpsFix& fix, uint32_t now_ms) {
    if (point_count_ == 0) {
        phase_ = RadarPhase::kIdle;
        return RadarEvent::kNone;
    }

    const bool usable =
        fix.valid && fix.satellites >= config_.min_satellites && fix.hdop <= config_.max_hdop;

    if (!usable) {
        // 精度不够时不推动状态机。否则在树下走两步，AI 就会喊「宝箱消失了」。
        if (last_valid_ms_ != 0 && (now_ms - last_valid_ms_) > kFixTimeoutMs) {
            phase_ = RadarPhase::kIdle;
            candidate_ = RadarPhase::kIdle;
            target_index_ = kNoTarget;
            ResetWindow();
            last_valid_ms_ = 0;
        }
        return RadarEvent::kNone;
    }
    last_valid_ms_ = now_ms;

    const GeoPoint here{fix.latitude, fix.longitude};

    // 找最近点位
    size_t nearest_index = 0;
    double nearest_distance = 1e18;
    for (size_t i = 0; i < point_count_; ++i) {
        const double d = DistanceMeters(here, points_[i].position);
        if (d < nearest_distance) {
            nearest_distance = d;
            nearest_index = i;
        }
    }

    // 目标锁定：已经锁定的点位，除非走远到 leave_radius 之外且明显有更近的，否则不换
    if (target_index_ == kNoTarget) {
        target_index_ = nearest_index;
        phase_ = RadarPhase::kFar;
        candidate_ = RadarPhase::kFar;
        candidate_since_ms_ = now_ms;
        ResetWindow();
    } else if (nearest_index != target_index_) {
        const double locked_distance =
            DistanceMeters(here, points_[target_index_].position);
        if (locked_distance > config_.leave_radius_m &&
            nearest_distance + 2.0 < locked_distance) {
            target_index_ = nearest_index;
            phase_ = RadarPhase::kFar;
            candidate_ = RadarPhase::kFar;
            candidate_since_ms_ = now_ms;
            ResetWindow();
            nearest_distance = DistanceMeters(here, points_[target_index_].position);
        }
    }

    const double distance = PushDistance(nearest_distance);
    smoothed_distance_m_ = distance;
    target_bearing_deg_ = BearingDegrees(here, points_[target_index_].position);

    RadarPhase observed = phase_;
    if (distance <= config_.arrive_radius_m) {
        observed = RadarPhase::kArrived;
    } else if (distance <= config_.approach_radius_m) {
        observed = RadarPhase::kNear;
    } else if (distance > config_.leave_radius_m) {
        observed = RadarPhase::kFar;
    }

    if (observed != candidate_) {
        candidate_ = observed;
        candidate_since_ms_ = now_ms;
        return RadarEvent::kNone;
    }

    if (observed == phase_) {
        return RadarEvent::kNone;
    }
    if ((now_ms - candidate_since_ms_) < config_.confirm_ms) {
        return RadarEvent::kNone;
    }

    const RadarPhase previous = phase_;
    phase_ = observed;

    if (phase_ == RadarPhase::kArrived) {
        return ShouldEmit(RadarEvent::kArrived, now_ms) ? RadarEvent::kArrived : RadarEvent::kNone;
    }
    if (phase_ == RadarPhase::kNear && previous == RadarPhase::kFar) {
        return ShouldEmit(RadarEvent::kApproaching, now_ms) ? RadarEvent::kApproaching
                                                            : RadarEvent::kNone;
    }
    if (phase_ == RadarPhase::kFar) {
        return ShouldEmit(RadarEvent::kLeft, now_ms) ? RadarEvent::kLeft : RadarEvent::kNone;
    }
    return RadarEvent::kNone;
}

}  // namespace radar
