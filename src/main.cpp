#include <Eigen/Core>
#include <iostream>
#include <vector>

#include <constants.hpp>
#include <helpers.hpp>
#include <io.hpp>
#include <motion.hpp>
#include <types.hpp>


int main(int argc, char** argv) {
  // read highway map
  map_t map = read_map(MAP_FILE);
  std::cout << "Loaded map with " << map.size << " waypoints." << std::endl;

  SimIO simulator(PORT, [&](car_t ego, path_t prev_path, double end_path_s,
      double end_path_d, std::vector<car_t> other_cars) {

    // generate path
    path_t path = move_in_lane(ego, 1, 20, prev_path, map);

    return path;
  });
  simulator.run();
  return 0;
}

