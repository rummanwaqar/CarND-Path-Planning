#include <Eigen/Core>
#include <iostream>
#include <vector>

#include <helpers.hpp>
#include <io.hpp>
#include <types.hpp>

const std::string MAP_FILE = "../data/highway_map.csv";
const int PORT = 4567;

int main(int argc, char** argv) {
  // read highway map
  map_t map = read_map(MAP_FILE);
  std::cout << "Loaded map with " << map.size << " waypoints." << std::endl;

  SimIO simulator(PORT, [&](car_t ego, path_t prev_path, double end_path_s,
      double end_path_d, std::vector<car_t> other_cars) {
    path_t path{};

    return path;
  });
  simulator.run();
  return 0;
}

