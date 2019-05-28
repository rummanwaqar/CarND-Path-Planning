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
    path_t path{};

    // stay in lane
    double dist = TIME_PER_STEP * (SPEED_LIMIT - 1);
    for(int i=0; i<50; i++) {
      double next_s = ego.s + (i + 1) * dist;
      double next_d = get_d_from_lane(1);
      double next_x, next_y;
      std::tie(next_x, next_y) = get_cartesian(next_s, next_d, map);
      path.x.push_back(next_x);
      path.y.push_back(next_y);
    }

    return path;
  });
  simulator.run();
  return 0;
}

