#pragma once

#include <string>

// map file
const std::string MAP_FILE = "../data/highway_map.csv";
// simulator port for connection
const int PORT = 4567;

// speed limit
const double SPEED_LIMIT = 22.0; // m/s = <50 mph

// acceleration
const double ACCELERATION = 0.224;

// lane change wait
const int LANE_CHANGE_COUNTER = 10;

// time simulator takes to move between two path points
const double TIME_PER_STEP = 0.02; // 20 ms

// width of each road lane
const double LANE_WIDTH = 4.0; // meters
// number of lanes on each side
const int MAX_LANES = 3;