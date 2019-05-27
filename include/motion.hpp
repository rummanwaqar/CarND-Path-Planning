#pragma once

#include <cmath>
#include "constants.hpp"
#include "types.hpp"

/*
 * transform path from relative to absolute coordinates
 * @param x: car x location
 * @param y: car y location
 * @param yaw: car yaw
 * @param path: path is transformed in place
 */
void transform_path_abs(double x, double y, double yaw, path_t& path);

/*
 * move in straight line at given speed
 * @param ego: our vehicle state
 * @param speed: target speed
 */
path_t move_straight(const car_t& ego, double speed);