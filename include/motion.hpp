#pragma once

#include <cmath>
#include <limits>
#include <tuple>
#include "constants.hpp"
#include "helpers.hpp"
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

/*
 * find the closest waypoint to specified x and y
 * @param x, y: find waypoint closest to x, y coods
 * @param map: waypoint map
 * @return index of closest waypoint
 */
int closest_waypoint(double x, double y, const map_t& map);

/*
 * find the closest NEXT waypoint to specified pose
 * @param x, y: find waypoint next to x, y coods
 * @param theta: direction of next
 * @param map: waypoint map
 * @return index of next waypoint
 */
int next_waypoint(double x, double y, double theta, const map_t& map);

/*
 * transform x, y to frenet s, d coods
 * @param x, y: cartesian coods
 * @param theta: angle of robot
 * @param map: waypoint map
 * @return s, d as tuple
 */
std::tuple<double, double> get_frenet(double x, double y, double theta, const map_t& map);

/*
 * transform frenet s, d to cartesian x, y coods
 * @param s, d: frenet coods
 * @param map: waypoint map
 * @return x, y as tuple
 */
std::tuple<double, double> get_cartesian(double s, double d, const map_t& map);

double get_d_from_lane(int lane);