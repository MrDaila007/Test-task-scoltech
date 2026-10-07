#pragma once

// Local NED -> WGS-84 on a flat-earth (equirectangular) approximation around the
// origin. Over the few hundred metres of an inspection flight the error is below
// a centimetre.

#include "fcstub/config.hpp"

#include <cmath>

namespace fcstub {

struct GeoPoint {
    double lat_deg;
    double lon_deg;
    double alt_msl_m;
};

inline GeoPoint ned_to_wgs84(const GeoOrigin& origin, double north_m, double east_m,
                             double down_m) noexcept {
    constexpr double kEarthRadius = 6371000.0;
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const double lat0_rad = origin.lat_deg / kRadToDeg;
    return {origin.lat_deg + north_m / kEarthRadius * kRadToDeg,
            origin.lon_deg + east_m / (kEarthRadius * std::cos(lat0_rad)) * kRadToDeg,
            origin.alt_msl_m - down_m};
}

}  // namespace fcstub
