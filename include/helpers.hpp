#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include "types.hpp"

/*
 * read highway map
 * @param map_file: map csv
 * @return map
 */
map_t read_map(std::string map_file) {
  map_t map{};
  std::ifstream in_file(map_file.c_str(), std::ifstream::in);
  if(!in_file) {
    throw std::runtime_error("Map file not found");
  }

  std::string line;
  while(getline(in_file, line)) {
    std::istringstream iss(line);
    double x, y;
    float s, dx, dy;
    iss >> x >> y >> s >> dx >> dy;
    map.x.push_back(x);
    map.y.push_back(y);
    map.s.push_back(s);
    map.dx.push_back(dx);
    map.dy.push_back(dy);
  }
  map.size = map.x.size();
  return map;
}