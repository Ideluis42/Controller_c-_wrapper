#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker.h"

geographic_msgs::GeoPoint _ekfOrigin;

void latlon_to_XY(double lat, double lon, double *X, double *Y) {
  // Convert lat/lon to NE
  static constexpr double LOCATION_SCALING_FACTOR = 111318.84502145034;  // deg to m
  static constexpr double DEG_TO_RAD = M_PI/180.0;

  *Y = (lat-_ekfOrigin.latitude) * LOCATION_SCALING_FACTOR;
  *X = (lon-_ekfOrigin.longitude) * LOCATION_SCALING_FACTOR * std::max(cos(_ekfOrigin.latitude*DEG_TO_RAD),0.01);  // not sure about this one, let's see how it actually goes...

  return;

}

void XY_to_latlon(double X, double Y, double *lat, double *lon) {
    // Convert NE to lat/lon
    static constexpr double LOCATION_SCALING_FACTOR_INV = 0.000008983204953368922;  // m to deg
    static constexpr double DEG_TO_RAD = M_PI/180.0;

    *lat = _ekfOrigin.latitude + Y*LOCATION_SCALING_FACTOR_INV;
    *lon = _ekfOrigin.longitude + X*LOCATION_SCALING_FACTOR_INV/std::max(cos(_ekfOrigin.latitude*DEG_TO_RAD),0.01);

    return;

}