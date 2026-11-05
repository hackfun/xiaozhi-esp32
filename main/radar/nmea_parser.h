#ifndef RADAR_NMEA_PARSER_H
#define RADAR_NMEA_PARSER_H

// ATGM336H 走 NMEA 0183，逐字节喂进来即可。
// 只解析两条真正需要的语句：
//   $xxRMC - 定位有效标志 + 坐标 + 速度 + 航向
//   $xxGGA - 卫星数 + HDOP + 定位质量（判断精度够不够用）
// 不依赖 ESP-IDF，可以在 PC 上单元测试。

#include <cstddef>
#include <cstdint>

namespace radar {

struct GpsFix {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    int satellites = 0;
    double hdop = 99.9;       // 水平精度因子，越小越准；>3 基本不能用
    double speed_kmh = 0.0;
    double course_deg = 0.0;
};

class NmeaParser {
public:
    static constexpr size_t kMaxSentence = 96;

    void Feed(char byte);
    void Feed(const char* data, size_t length);
    // 测试与调试用：喂一整句（含 $ 和 *hh），返回是否解析成功
    bool FeedSentence(const char* sentence);

    const GpsFix& fix() const { return fix_; }
    void Reset();

private:
    void HandleSentence(const char* sentence);
    void ParseRmc(char* fields[], int count);
    void ParseGga(char* fields[], int count);

    // 留一个字节给结尾 '\0'
    char buffer_[kMaxSentence + 1] = {};
    size_t length_ = 0;
    bool overflow_ = false;
    GpsFix fix_;
};

}  // namespace radar

#endif  // RADAR_NMEA_PARSER_H
