#include "motion.hpp"

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

path_t move_in_lane(const car_t &ego, int lane, double speed, const path_t &prev_path, const map_t &map) {
  // generate a sparse path
  path_t sparse_path{};

  // initial state (either from current location or previous path)
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  size_t prev_path_size = prev_path.x.size();
  // generate first two points using car's location or prev path
  if(prev_path_size < 2) {
    // previous path is close to empty
    // use two points tangent to car
    sparse_path.x.push_back(ref_x - ::cos(ref_yaw));
    sparse_path.y.push_back(ref_y - ::sin(ref_yaw));

    sparse_path.x.push_back(ref_x);
    sparse_path.y.push_back(ref_y);
  } else {
    // use previous path as starting condition
    ref_x = prev_path.x[prev_path_size - 1];
    ref_y = prev_path.y[prev_path_size - 1];
    double ref_x_prev = prev_path.x[prev_path_size - 2];
    double ref_y_prev = prev_path.y[prev_path_size - 2];
    ref_yaw = ::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    sparse_path.x.push_back(ref_x_prev);
    sparse_path.y.push_back(ref_y_prev);

    sparse_path.x.push_back(ref_x);
    sparse_path.y.push_back(ref_y);
  }

  // in frenet generate 3 more points 30 meters apart
  for(int i=1; i<=3; i++) {
    auto wp = get_cartesian(ego.s + 30 * i, get_d_from_lane(lane), map);
    sparse_path.x.push_back(wp.first);
    sparse_path.y.push_back(wp.second);
  }

  // move path to vehicle frame
  transform_path_rel(ref_x, ref_y, ref_yaw, sparse_path);

  // create spline with sparse path
  tk::spline s;
  s.set_points(sparse_path.x, sparse_path.y);

  // generate smooth path from spline
  path_t path;
  // add all points from previous path
  for(int i=0; i<prev_path_size; i++) {
    path.x.push_back(prev_path.x[i]);
    path.y.push_back(prev_path.y[i]);
  }
  // calc how to break up the spline for target vel
  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = ::sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (TIME_PER_STEP * speed);

  double x_add_on = 0;
  // fill the remaining path with new points
  for(int i=0; i< 50 - prev_path_size; i++) {
    std::pair<double, double> point;
    point.first = x_add_on + target_x / N;
    point.second = s(point.first);
    x_add_on = point.first;
    // convert to abs coods
    transform_point_abs(ref_x, ref_y, ref_yaw, point);
    path.x.push_back(point.first);
    path.y.push_back(point.second);
  }

  return path;
}

bool check_collision_in_lane(int lane, double ego_s, double end_path_s, int prev_path_size, const std::vector<car_t> &other_cars) {
  if(prev_path_size > 0) {
    ego_s = end_path_s;
  }
  // cycle through all the other cars
  for(const auto car : other_cars) {
    // check lane
    if(car.d < (get_d_from_lane(lane) + 2) && car.d > (get_d_from_lane(lane) - 2)) {
      double speed = ::sqrt(car.vel_x * car.vel_x + car.vel_y * car.vel_y);
      double check_s = car.s;
      check_s += static_cast<double>(prev_path_size) * TIME_PER_STEP * speed;
      if(check_s > ego_s && (check_s - ego_s) < 30) {
        return true;
      }
    }
  };
  return false;
}

bool check_lane_for_switch(int lane, double ego_s, double end_path_s, int prev_path_size, const std::vector<car_t>& other_cars, double safe_distance) {
  if(lane <= 0 || lane > MAX_LANES) {
    return false;
  }
  if(prev_path_size > 0) {
    ego_s = end_path_s;
  }
  // cycle through all the other cars
  for(const auto car : other_cars) {
    // check lane
    if(car.d < (get_d_from_lane(lane) + 2) && car.d > (get_d_from_lane(lane) - 2)) {
      double speed = ::sqrt(car.vel_x * car.vel_x + car.vel_y * car.vel_y);
      double check_s = car.s;
      check_s += static_cast<double>(prev_path_size) * TIME_PER_STEP * speed;
      if((check_s - ego_s) < safe_distance && (ego_s - check_s) < safe_distance) {
        return false;
      }
    }
  };
  return true;
}

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
    double temp_x = cy * path.x[i] - sy * path.y[i] + x;
    double temp_y = sy * path.x[i] + cy * path.y[i] + y;
    path.x[i] = temp_x;
    path.y[i] = temp_y;
  }
}

void transform_point_abs(double ref_x, double ref_y, double ref_yaw, std::pair<double, double> &point) {
  /*
   * abs = T * rel
   * where transformation matrix T
   * cos(theta), -sin(theta), x
   * sin(theta), cos(theta), y
   * 0, 0, 1
   */
  double cy = ::cos(ref_yaw);
  double sy = ::sin(ref_yaw);
  double temp_x = cy * point.first - sy * point.second + ref_x;
  double temp_y = sy * point.first + cy * point.second + ref_y;
  point.first = temp_x;
  point.second = temp_y;
}


void transform_path_rel(double x, double y, double yaw, path_t& path) {
  /*
   * transform to relative coods
   * multiply each waypoint by inverse of ref pose matrix
   * [[cos(θ), -sin(θ), ref_x], [sin(θ), cos(θ), ref_y], [0,0,1]].inverse()
   */
  for(int i=0; i<path.x.size(); i++) {
    double shift_x = path.x[i] - x;
    double shift_y = path.y[i] - y;
    path.x[i] = shift_x * cos(yaw) + shift_y * sin(yaw);
    path.y[i] = -shift_x * sin(yaw) + shift_y * cos(yaw);
  }
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

std::pair<double, double> get_frenet(double x, double y, double theta, const map_t &map) {
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

  return {frenet_s, frenet_d};
}

std::pair<double, double> get_cartesian(double s, double d, const map_t &map) {
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

  return {x, y};
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

