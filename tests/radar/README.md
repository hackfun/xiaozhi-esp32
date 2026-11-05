# 寻宝雷达算法测试

覆盖 `main/radar/` 下三个不依赖 ESP-IDF 的模块：

| 文件 | 测什么 |
|---|---|
| `geo_utils.cc` | 球面距离、方位角、平面偏移 |
| `nmea_parser.cc` | RMC / GGA 解析、校验和、无效定位、流式喂入 |
| `treasure_engine.cc` | 三态状态机、中位数滤波、迟滞、确认时长、按事件类型冷却 |

`radar_service.cc` 和 `radar_tools.cc` 依赖固件运行时（Application / McpServer），
只能上板验证，不在这个测试范围内。

## 怎么跑

本机没有 C++ 编译器不要紧，CI 会跑（`.github/workflows/build.yml` 的
`Test radar algorithm (host)` 步骤）。本地有 g++ 时：

```bash
cd tools/xiaozhi-esp32
g++ -std=c++17 -O1 -Wall -Wextra -Imain \
  tests/radar/test_radar.cc \
  main/radar/geo_utils.cc \
  main/radar/nmea_parser.cc \
  main/radar/treasure_engine.cc \
  -o /tmp/test_radar
/tmp/test_radar
```

通过时输出 `N checks, 0 failures`，退出码 0。

## 期望值从哪来

不是手填的。`reference.py` 是同一套算法的独立 Python 实现，先在上面跑轨迹，
把实际事件序列打印出来，再写进 C++ 断言。这样能提前发现「帧数算错」这类问题——
第一版就踩过：`approach` 实际要第 6 帧才触发，而不是想当然的第 4 帧。

改算法时两边要一起改：

```bash
python tests/radar/reference.py
```

## 测试里的几个关键约束

- **校验和错误必须整句丢弃**：串口线路噪声很常见，脏数据进状态机会导致坐标乱跳。
- **单帧毛刺不能误报离开**：中位数窗口 5 帧，一帧跳到 40m 会被吃掉。
- **卫星数不足时不推动状态机**：否则在树下走两步，AI 就喊「宝箱消失了」。
- **冷却按事件类型分开计**：早期版本用单一冷却，导致「到达」把随后的「离开」一起吞了。
