#include <iostream>
//#include <Eigen/Core>

#include <helpers.hpp>
#include <io.hpp>
#include <types.hpp>

const std::string MAP_FILE = "../data/highway_map.csv";
const int PORT = 4567;

int main(int argc, char** argv) {
  // read highway map
  map_t map = read_map(MAP_FILE);
  std::cout << "Loaded map with " << map.size << " waypoints." << std::endl;
  return 0;
}

