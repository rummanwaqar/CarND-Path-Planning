#pragma once

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "types.hpp"

/*
 * read highway map
 * @param map_file: map csv
 * @return map
 */
map_t read_map(std::string map_file);

/*
 * convert deg to radian
 */
inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

/*
 * convert rad to deg
 */
inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}