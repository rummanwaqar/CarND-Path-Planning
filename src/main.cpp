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

  int lane = 2;
  double ego_speed = 0;

  SimIO simulator(PORT, [&](car_t ego, path_t prev_path, double end_path_s,
      double end_path_d, std::vector<car_t> other_cars) {

    // check for collision
    bool too_close = check_collision_in_lane(lane, ego.s, end_path_s, prev_path.x.size(), other_cars);

    // adjust speed
    if(too_close) {
      ego_speed -= ACCELERATION;
      // check if we can turn
      if(check_lane_for_switch(lane - 1, ego.s, end_path_s, prev_path.x.size(), other_cars, 15)) {
        lane -= 1;
      } else if(check_lane_for_switch(lane + 1, ego.s, end_path_s, prev_path.x.size(), other_cars, 15)) {
        lane += 1;
      }
    } else if (ego_speed < SPEED_LIMIT){
      ego_speed += ACCELERATION;
    }

    // generate path
    std::cout << lane << std::endl;
    path_t path = move_in_lane(ego, lane, ego_speed, prev_path, map);

    return path;
  });
  simulator.run();
  return 0;
}

