#pragma once

#include <string>

// map file
const std::string MAP_FILE = "../data/highway_map.csv";
// simulator port for connection
const int PORT = 4567;

// speed limit
const double SPEED_LIMIT = 22.3; // m/s = 50 mph

// time simulator takes to move between two path points
const double TIME_PER_STEP = 0.02; // 20 ms