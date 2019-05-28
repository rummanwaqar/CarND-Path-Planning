#include "motion.hpp"

void transform_path_abs(double x, double y, double yaw, path_t& path) {
  /*
   * abs = T * rel
   * where transformation matrix T
   * cos(theta), -sin(theta), x
   * sin(theta), cos(theta), y
   * 0, 0, 1
   */
  double cy = ::cos(yaw);
  double sy = ::sin(yaw);
  for(int i=0; i<path.x.size(); i++) {
    path.x[i] = cy * path.x[i] - sy * path.y[i] + x;
    path.y[i] = sy * path.x[i] + cy * path.y[i] + y;
  }
}

path_t move_straight(const car_t& ego, double speed) {
  path_t path{};
  // longitudinal distance between points
  double dist = TIME_PER_STEP * speed;
  // generate relative path
  for(int i=0; i<50; i++) {
    path.x.push_back(dist * i);
    path.y.push_back(0);
  }
  // transform path to abs coordinates
  transform_path_abs(ego.x, ego.y, deg2rad(ego.yaw), path);
  return path;
}
