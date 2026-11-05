#ifndef RADAR_GEO_UTILS_H
#define RADAR_GEO_UTILS_H

// 纯几何计算，不依赖 ESP-IDF，可以在 PC 上直接单元测试。

namespace radar {

constexpr double kPi = 3.14159265358979323846;
// WGS84 平均地球半径
constexpr double kEarthRadiusMeters = 6371000.0;

struct GeoPoint {
    double latitude = 0.0;   // 十进制度，北纬为正
    double longitude = 0.0;  // 十进制度，东经为正
};

inline double DegToRad(double degree) { return degree * kPi / 180.0; }

// 球面距离（Haversine）。寻宝场景点位通常在 50km 内，误差可忽略。
double DistanceMeters(const GeoPoint& from, const GeoPoint& to);

// 从 from 看向 to 的真北方位角，0~360。
double BearingDegrees(const GeoPoint& from, const GeoPoint& to);

// 把目标点折算成以 origin 为原点的平面偏移（米）。
// east 向东为正，north 向北为正。用于屏幕上的箭头/点位绘制。
void OffsetMeters(const GeoPoint& origin, const GeoPoint& target, double& east, double& north);

}  // namespace radar

#endif  // RADAR_GEO_UTILS_H
