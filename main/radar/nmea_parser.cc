#include "radar/nmea_parser.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace radar {
namespace {

bool HexValue(char c, int& out) {
    if (c >= '0' && c <= '9') {
        out = c - '0';
        return true;
    }
    if (c >= 'A' && c <= 'F') {
        out = c - 'A' + 10;
        return true;
    }
    if (c >= 'a' && c <= 'f') {
        out = c - 'a' + 10;
        return true;
    }
    return false;
}

// 校验 '$' 与 '*' 之间的逐字节异或
bool ChecksumOk(const char* sentence) {
    const char* star = std::strchr(sentence, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0') {
        return false;
    }
    int high = 0;
    int low = 0;
    if (!HexValue(star[1], high) || !HexValue(star[2], low)) {
        return false;
    }
    const uint8_t expected = static_cast<uint8_t>((high << 4) | low);
    uint8_t actual = 0;
    for (const char* p = sentence + 1; p < star; ++p) {
        actual ^= static_cast<uint8_t>(*p);
    }
    return expected == actual;
}

// NMEA 坐标是 ddmm.mmmm / dddmm.mmmm（度分），转成十进制度
bool ParseCoordinate(const char* value, const char* hemisphere, double& out) {
    if (value == nullptr || value[0] == '\0' || hemisphere == nullptr || hemisphere[0] == '\0') {
        return false;
    }
    char* end = nullptr;
    const double raw = std::strtod(value, &end);
    if (end == value) {
        return false;
    }
    const double degrees = std::floor(raw / 100.0);
    const double minutes = raw - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    out = degrees + minutes / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        out = -out;
    }
    return true;
}

}  // namespace

void NmeaParser::Feed(char byte) {
    if (byte == '\r') {
        return;
    }
    if (byte == '\n') {
        if (length_ > 0 && !overflow_) {
            buffer_[length_] = '\0';
            HandleSentence(buffer_);
        }
        length_ = 0;
        overflow_ = false;
        return;
    }
    if (byte == '$') {
        // 遇到新的起始符，丢掉未完成的残句重新开始
        length_ = 0;
        overflow_ = false;
        buffer_[length_++] = byte;
        return;
    }
    if (length_ < kMaxSentence) {
        buffer_[length_++] = byte;
    } else {
        overflow_ = true;
    }
}

void NmeaParser::Feed(const char* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        Feed(data[i]);
    }
}

bool NmeaParser::FeedSentence(const char* sentence) {
    if (sentence == nullptr) {
        return false;
    }
    GpsFix before = fix_;
    HandleSentence(sentence);
    const bool changed = (before.valid != fix_.valid) ||
                         (before.latitude != fix_.latitude) ||
                         (before.longitude != fix_.longitude) ||
                         (before.satellites != fix_.satellites);
    return changed;
}

void NmeaParser::Reset() {
    length_ = 0;
    overflow_ = false;
    fix_ = GpsFix{};
}

void NmeaParser::HandleSentence(const char* sentence) {
    if (sentence == nullptr || sentence[0] != '$') {
        return;
    }
    if (!ChecksumOk(sentence)) {
        return;
    }

    char work[kMaxSentence + 1];
    std::strncpy(work, sentence, kMaxSentence);
    work[kMaxSentence] = '\0';

    // 切掉校验和，只留数据部分
    char* star = std::strchr(work, '*');
    if (star != nullptr) {
        *star = '\0';
    }

    // 就地按逗号切分。fields[0] 形如 "$GNRMC" / "$GPRMC"
    char* fields[24] = {};
    int count = 0;
    fields[count++] = work;
    for (char* p = work; *p != '\0' && count < 24; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    // 跳过 "$GN" / "$GP" / "$BD" 两个字符的 talker id
    const char* type = fields[0] + 3;
    if (std::strcmp(type, "RMC") == 0) {
        ParseRmc(fields, count);
    } else if (std::strcmp(type, "GGA") == 0) {
        ParseGga(fields, count);
    }
}

void NmeaParser::ParseRmc(char* fields[], int count) {
    // $--RMC,time,status,lat,N/S,lon,E/W,speed_knots,course,date,...
    if (count < 9) {
        return;
    }
    if (fields[2][0] != 'A') {
        // V = 无效定位（冷启动、室内、天线断开）
        fix_.valid = false;
        return;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!ParseCoordinate(fields[3], fields[4], latitude)) {
        return;
    }
    if (!ParseCoordinate(fields[5], fields[6], longitude)) {
        return;
    }

    fix_.latitude = latitude;
    fix_.longitude = longitude;
    fix_.valid = true;

    if (fields[7][0] != '\0') {
        // 原始单位是节
        fix_.speed_kmh = std::strtod(fields[7], nullptr) * 1.852;
    }
    if (fields[8][0] != '\0') {
        fix_.course_deg = std::strtod(fields[8], nullptr);
    }
}

void NmeaParser::ParseGga(char* fields[], int count) {
    // $--GGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,alt,...
    if (count < 9) {
        return;
    }
    const int quality = std::atoi(fields[6]);
    if (quality == 0) {
        fix_.valid = false;
        return;
    }

    fix_.satellites = std::atoi(fields[7]);
    if (fields[8][0] != '\0') {
        fix_.hdop = std::strtod(fields[8], nullptr);
    }

    // 坐标以 RMC 为准；GGA 先到时用它兜底，避免开头几秒没有位置
    if (!fix_.valid) {
        double latitude = 0.0;
        double longitude = 0.0;
        if (ParseCoordinate(fields[2], fields[3], latitude) &&
            ParseCoordinate(fields[4], fields[5], longitude)) {
            fix_.latitude = latitude;
            fix_.longitude = longitude;
            fix_.valid = true;
        }
    }
}

}  // namespace radar
