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

int closest_waypoint(double x, double y, const map_t &map) {
  double closest_len = std::numeric_limits<double>::max();
  int closest_index = 0;

  for(int i=0; i<map.size; i++) {
    double dist = distance(x, y, map.x[i], map.y[i]);
    if(dist < closest_len) {
      closest_len = dist;
      closest_index = i;
    }
  }
  return closest_index;
}

int next_waypoint(double x, double y, double theta, const map_t &map) {
  int closest_index = closest_waypoint(x, y, map);
  double map_x = map.x[closest_index];
  double map_y = map.y[closest_index];
  double heading = ::atan2(map_y - y, map_x - x);
  if(::fabs(theta - heading) > M_PI_4) {
    closest_index++;
    if(closest_index >= map.size) {
      closest_index = 0;
    }
  }
  return closest_index;
}

std::tuple<double, double> get_frenet(double x, double y, double theta, const map_t &map) {
  int next_wp = next_waypoint(x, y, theta, map);
  int prev_wp = next_wp - 1;
  if(next_wp == 0) {
    prev_wp = map.size - 1;
  }

  double n_x = map.x[next_wp] - map.x[prev_wp];
  double n_y = map.y[next_wp] - map.y[prev_wp];
  double x_x = x - map.x[prev_wp];
  double x_y = y - map.y[prev_wp];
  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - map.x[prev_wp];
  double center_y = 2000 - map.y[prev_wp];
  double center_to_pos = distance(center_x, center_y, x_x, x_y);
  double center_to_ref = distance(center_x, center_y, proj_x, proj_y);
  if (center_to_pos <= center_to_ref) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(map.x[i], map.y[i], map.x[i+1], map.y[i+1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);

  return std::make_tuple(frenet_s, frenet_d);
}

std::tuple<double, double> get_cartesian(double s, double d, const map_t &map) {
  // find previous and next waypoint
  int prev_wp = -1;
  while(s > map.s[prev_wp + 1] && (prev_wp < static_cast<int>(map.size - 1))) {
    ++prev_wp;
  }
  int next_wp = (prev_wp + 1) % map.size;

  // find angle between two waypoints
  double heading_wp = ::atan2((map.y[next_wp] - map.y[prev_wp]),
      (map.x[next_wp] - map.x[prev_wp]));

  // x, y, s along the segment
  double seg_s = s - map.s[prev_wp];
  double seg_x = map.x[prev_wp] + seg_s * cos(heading_wp);
  double seg_y = map.y[prev_wp] + seg_s * sin(heading_wp);

  // perpendicular shift caused by d
  double perp_heading_wp = heading_wp - M_PI_2;
  double x = seg_x + d * cos(perp_heading_wp);
  double y = seg_y + d * sin(perp_heading_wp);

  return std::make_tuple(x, y);
}

double get_d_from_lane(int lane) {
  if(lane > 0 && lane <= MAX_LANES) {
    return (lane * LANE_WIDTH) - LANE_WIDTH / 2.0;
  } else if(lane < 0 && lane <= -MAX_LANES) {
    return (lane * LANE_WIDTH) + LANE_WIDTH / 2.0;
  } else {
    throw std::runtime_error("Invalid lane");
  }
}

