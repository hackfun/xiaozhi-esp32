#include "radar/geo_utils.h"

#include <cmath>

namespace radar {

double DistanceMeters(const GeoPoint& from, const GeoPoint& to) {
    const double lat1 = DegToRad(from.latitude);
    const double lat2 = DegToRad(to.latitude);
    const double delta_lat = lat2 - lat1;
    const double delta_lon = DegToRad(to.longitude - from.longitude);

    const double sin_lat = std::sin(delta_lat * 0.5);
    const double sin_lon = std::sin(delta_lon * 0.5);
    const double a = sin_lat * sin_lat + std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
    // 数值误差可能让 a 略微超过 1，夹一下避免 sqrt 负数
    const double clamped = (a > 1.0) ? 1.0 : a;
    const double c = 2.0 * std::atan2(std::sqrt(clamped), std::sqrt(1.0 - clamped));
    return kEarthRadiusMeters * c;
}

double BearingDegrees(const GeoPoint& from, const GeoPoint& to) {
    const double lat1 = DegToRad(from.latitude);
    const double lat2 = DegToRad(to.latitude);
    const double delta_lon = DegToRad(to.longitude - from.longitude);

    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
                     std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    double degrees = std::atan2(y, x) * 180.0 / kPi;
    if (degrees < 0.0) {
        degrees += 360.0;
    }
    return degrees;
}

void OffsetMeters(const GeoPoint& origin, const GeoPoint& target, double& east, double& north) {
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon = kMetersPerDegreeLat * std::cos(DegToRad(origin.latitude));
    north = (target.latitude - origin.latitude) * kMetersPerDegreeLat;
    east = (target.longitude - origin.longitude) * meters_per_degree_lon;
}

}  // namespace radar
