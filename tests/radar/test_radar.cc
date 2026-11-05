// 寻宝核心算法的 host 端单元测试。
// 不依赖 ESP-IDF，编译方式见 tests/radar/README.md。
// 期望值来自 reference.py 的独立实现（Python），不是拍脑袋填的。

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "radar/geo_utils.h"
#include "radar/nmea_parser.h"
#include "radar/radar_render.h"
#include "radar/treasure_engine.h"

using namespace radar;

namespace {

int g_checks = 0;
int g_failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        ++g_checks;                                                     \
        if (!(cond)) {                                                  \
            ++g_failures;                                               \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                               \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                         \
    do {                                                                          \
        ++g_checks;                                                               \
        const double actual_ = (actual);                                          \
        const double expected_ = (expected);                                      \
        if (std::fabs(actual_ - expected_) > (tol)) {                             \
            ++g_failures;                                                         \
            std::printf("FAIL %s:%d  %s = %.4f, expected %.4f (tol %.4f)\n",      \
                        __FILE__, __LINE__, #actual, actual_, expected_,          \
                        static_cast<double>(tol));                                \
        }                                                                         \
    } while (0)

void TestNmea() {
    NmeaParser parser;
    CHECK(parser.FeedSentence(
        "$GNRMC,085618.000,A,3941.1234,N,11623.5678,E,0.02,0.00,051126,,,A*71"));
    CHECK(parser.FeedSentence(
        "$GPGGA,085618.000,3941.1234,N,11623.5678,E,1,09,0.85,45.2,M,0.0,M,,*69"));

    const GpsFix& fix = parser.fix();
    CHECK(fix.valid);
    CHECK_NEAR(fix.latitude, 39.685390, 1e-5);
    CHECK_NEAR(fix.longitude, 116.392797, 1e-5);
    CHECK(fix.satellites == 9);
    CHECK_NEAR(fix.hdop, 0.85, 1e-3);
    // 0.02 节 ≈ 0.037 km/h
    CHECK_NEAR(fix.speed_kmh, 0.03704, 1e-3);

    // 校验和错误的是脏数据，必须整句丢掉
    NmeaParser corrupted;
    corrupted.FeedSentence("$GNRMC,085618.000,A,3941.1234,N,11623.5678,E,0.02,0.00,051126,,,A*00");
    CHECK(!corrupted.fix().valid);

    // V = 无效定位（室内 / 冷启动 / 天线没接好）
    NmeaParser invalid;
    invalid.FeedSentence("$GNRMC,085618.000,V,,,,,,,051126,,,N*50");
    CHECK(!invalid.fix().valid);

    // 逐字节喂入应当与整句喂入等价（串口是流式到达的）
    NmeaParser streamed;
    const char* sentence = "$GNRMC,085618.000,A,3941.1234,N,11623.5678,E,0.02,0.00,051126,,,A*71\r\n";
    streamed.Feed(sentence, std::strlen(sentence));
    CHECK(streamed.fix().valid);
    CHECK_NEAR(streamed.fix().latitude, 39.685390, 1e-5);
}

void TestDistance() {
    const GeoPoint here{39.685390, 116.392797};
    CHECK_NEAR(DistanceMeters(here, here), 0.0, 0.001);

    const GeoPoint tiananmen{39.9087, 116.3975};
    const GeoPoint taihedian{39.9163, 116.3906};
    CHECK_NEAR(DistanceMeters(tiananmen, taihedian), 1029.80, 2.0);

    double east = 0.0;
    double north = 0.0;
    const GeoPoint north_100m{39.685390 + 100.0 / 111320.0, 116.392797};
    OffsetMeters(here, north_100m, east, north);
    CHECK_NEAR(north, 100.0, 0.5);
    CHECK_NEAR(east, 0.0, 1.0);
}

void TestBearing() {
    const GeoPoint origin{39.0, 116.0};
    CHECK_NEAR(BearingDegrees(origin, GeoPoint{39.01, 116.0}), 0.0, 0.5);
    CHECK_NEAR(BearingDegrees(origin, GeoPoint{39.0, 116.01}), 90.0, 0.5);
}

void TestStateMachine() {
    const GeoPoint target{39.685390, 116.392797};
    const TreasurePoint points[] = {{"t1", "测试宝箱", target}};

    TreasureEngine engine;
    engine.SetPoints(points, 1);
    engine.SetConfig(RadarConfig{});

    // 目标正北 distance_m 米处的定位
    auto make_fix = [&target](double distance_m, int satellites = 9, double hdop = 0.85) {
        GpsFix fix;
        fix.valid = true;
        fix.satellites = satellites;
        fix.hdop = hdop;
        fix.latitude = target.latitude + distance_m / 111320.0;
        fix.longitude = target.longitude;
        return fix;
    };

    uint32_t now = 1000;

    // 远处 50m：不该有任何事件
    for (int i = 0; i < 5; ++i, now += 1000) {
        CHECK(engine.Update(make_fix(50.0), now) == RadarEvent::kNone);
    }

    // 进入 14m：连续满足 confirm_ms 之后才报「接近」（参考实现实测第 6 帧触发）
    RadarEvent approaching = RadarEvent::kNone;
    for (int i = 0; i < 8 && approaching == RadarEvent::kNone; ++i, now += 1000) {
        approaching = engine.Update(make_fix(14.0), now);
    }
    CHECK(approaching == RadarEvent::kApproaching);

    // 进入 4m：报「到达」
    RadarEvent arrived = RadarEvent::kNone;
    for (int i = 0; i < 10 && arrived == RadarEvent::kNone; ++i, now += 1000) {
        arrived = engine.Update(make_fix(4.0), now);
    }
    CHECK(arrived == RadarEvent::kArrived);
    CHECK(engine.phase() == RadarPhase::kArrived);

    // 单帧毛刺跳到 40m：中位数滤波 + 迟滞应当把它吃掉，不许报「离开」
    CHECK(engine.Update(make_fix(40.0), now) == RadarEvent::kNone);
    now += 1000;
    CHECK(engine.phase() == RadarPhase::kArrived);

    // 持续走远 45m：最终要报「离开」
    RadarEvent left = RadarEvent::kNone;
    for (int i = 0; i < 12 && left == RadarEvent::kNone; ++i, now += 1000) {
        left = engine.Update(make_fix(45.0), now);
    }
    CHECK(left == RadarEvent::kLeft);

    // 精度不够（卫星数不足）时不许推动状态机
    TreasureEngine weak;
    weak.SetPoints(points, 1);
    weak.SetConfig(RadarConfig{});
    weak.Update(make_fix(50.0, 9), 1000);
    CHECK(weak.Update(make_fix(4.0, 2, 8.0), 2000) == RadarEvent::kNone);
    CHECK(weak.phase() != RadarPhase::kArrived);
}

void TestRender() {
    const int w = 240;
    const int h = 240;
    std::vector<uint16_t> static_buf(static_cast<size_t>(w) * h, 0);
    std::vector<uint16_t> frame_buf(static_cast<size_t>(w) * h, 0);

    RenderStatic(static_buf.data(), w, h, RadarRenderStyle{});

    // 四角在圆外，必须是纯背景色（圆屏上这四个角本来也不可见）
    CHECK(static_buf[0] == kColorBackground);
    CHECK(static_buf[static_cast<size_t>(h - 1) * w + (w - 1)] == kColorBackground);
    // 圆心被十字线覆盖
    CHECK(static_buf[(h / 2) * w + (w / 2)] != kColorBackground);

    RadarRenderInput input;
    input.width = w;
    input.height = h;
    input.sweep_angle_deg = 90.0f;  // 正右
    input.distance_m = 3.0f;
    input.proximity = 0.9f;
    input.pulse_phase = 0.5f;
    RenderFrame(frame_buf.data(), w, h, static_buf.data(), input, RadarRenderStyle{});

    // 角度 0 应当指向正上方、90 指向正右。若坐标系搞反，这条会挂。
    bool right_lit = false;
    const int mid_y = h / 2;
    for (int x = w / 2 + 15; x < w / 2 + 50; ++x) {
        if (frame_buf[static_cast<size_t>(mid_y) * w + x] !=
            static_buf[static_cast<size_t>(mid_y) * w + x]) {
            right_lit = true;
            break;
        }
    }
    CHECK(right_lit);

    bool top_lit = false;
    const int mid_x = w / 2;
    for (int y = h / 2 - 50; y < h / 2 - 15; ++y) {
        if (frame_buf[static_cast<size_t>(y) * w + mid_x] !=
            static_buf[static_cast<size_t>(y) * w + mid_x]) {
            top_lit = true;
            break;
        }
    }
    CHECK(!top_lit);  // 90 度时正上方不该有扫描线

    // 无目标时不应画信号点
    input.distance_m = -1.0f;
    input.proximity = 0.0f;
    RenderFrame(frame_buf.data(), w, h, static_buf.data(), input, RadarRenderStyle{});
    CHECK(frame_buf[(h / 2) * w + (w / 2)] != kColorBackground);
}

}  // namespace

int main() {
    TestNmea();
    TestDistance();
    TestBearing();
    TestStateMachine();
    TestRender();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
