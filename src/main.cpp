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
//    path_t path{};

    std::cout << "x,y: " << ego.x << "," << ego.y << "\t"
              << "s,d: " << ego.s << "," << ego.d << "\t"
              << "yaw: " << ego.yaw << "\t"
              << "speed: " << ego.speed << "\t"
              << "vel x,y: " << ego.vel_x << "," << ego.vel_y
              << std::endl;

    return move_straight(ego, SPEED_LIMIT);
//    return path;
  });
  simulator.run();
  return 0;
}

