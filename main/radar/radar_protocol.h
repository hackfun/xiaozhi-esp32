#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H

// 手表与掌机之间的 ESP-NOW 报文定义。
// 掌机侧需要同一份定义，改动时两边同步。

#include <cstdint>

#define RADAR_PROTOCOL_MAGIC   0xA5
#define RADAR_PROTOCOL_VERSION 1

enum RadarPacketType : uint8_t {
    kRadarPacketEvent = 1,  // 手表 -> 掌机：寻宝事件，掌机据此开口
    kRadarPacketPing = 2,   // 掌机 -> 手表：我在。手表用它测 RSSI 反过来找掌机
};

enum RadarPhaseCode : uint8_t {
    kPhaseIdle = 0,
    kPhaseFar = 1,
    kPhaseNear = 2,
    kPhaseArrived = 3,
};

// 固定 32 字节，ESP-NOW 单包上限 250，够用且方便对齐
typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t phase;
    uint8_t satellites;
    uint8_t reserved;
    uint16_t distance_cm;  // 0~655m，厘米精度对这个场景绰绰有余
    uint16_t point_id;
    char point_name[24];   // UTF-8，不保证以 '\0' 结尾时按 24 字节读完
} radar_packet_t;

static_assert(sizeof(radar_packet_t) == 34, "radar_packet_t 布局变了，两边要同步");

#endif  // RADAR_PROTOCOL_H
