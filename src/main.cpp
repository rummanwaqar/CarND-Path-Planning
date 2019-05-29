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
  int prepare_lane_change_left = 0;
  int prepare_lane_change_right = 0;

  SimIO simulator(PORT, [&](car_t ego, path_t prev_path, double end_path_s,
      double end_path_d, std::vector<car_t> other_cars) {

    // only consider atmost 10 element of the prev path
    if(prev_path.x.size() > 10) {
      prev_path.x.resize(10);
      prev_path.y.resize(10);
    }

    // check for collision
    bool car_ahead = check_collision_in_lane(lane, ego.s, end_path_s, prev_path.x.size(), other_cars);

    if(car_ahead) {
      ego_speed -= ACCELERATION;
      // check if we can switch lane
      if(check_lane_for_switch(lane - 1, ego.s, end_path_s, prev_path.x.size(), other_cars, 30)) {
        prepare_lane_change_left++;
        prepare_lane_change_right = 0;
      } else if(check_lane_for_switch(lane + 1, ego.s, end_path_s, prev_path.x.size(), other_cars, 30)) {
        prepare_lane_change_right++;
        prepare_lane_change_left = 0;
      } else {
        prepare_lane_change_left = 0;
        prepare_lane_change_right = 0;
      }
    } else if (ego_speed < SPEED_LIMIT){
      ego_speed += ACCELERATION;
    }

    if(prepare_lane_change_left == LANE_CHANGE_COUNTER) {
      lane -= 1;
      prepare_lane_change_left = 0;
    }
    if(prepare_lane_change_right == LANE_CHANGE_COUNTER) {
      lane += 1;
      prepare_lane_change_right = 0;
    }

    // generate path
    path_t path = move_in_lane(ego, lane, ego_speed, prev_path, map);

    return path;
  });
  simulator.run();
  return 0;
}

