#pragma once

#include <vector>

// represents car localization information
struct car_t {
  int id;       // cars unique id - used for other vehicles
  double x;     // x-position in map coods
  double y;     // y-position in map coods
  double s;     // s-position in fernet coods
  double d;     // d-position in fernet coods
  double yaw;   // yaw angle of car in map (degrees)
  double speed; // speed of car (MPH)
  double vel_x;  // x-velocity (m/s)
  double vel_y;  // y-velocity (m/s)
};

// represents path (x,y)
struct path_t {
  std::vector<double> x; // path x coods
  std::vector<double> y; // path y coods
};

// highway map
struct map_t {
  std::vector<double> x;  // x position of way point
  std::vector<double> y;  // y position of way point
  std::vector<double> s;  // distance along road to get to next waypoint
  // dx and dy define the unit normal vector pointing outward of the highway loop
  std::vector<double> dx;
  std::vector<double> dy;
  size_t size;            // number of waypoints in map
};
