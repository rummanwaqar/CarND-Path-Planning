#pragma once

#include <cmath>
#include <limits>
#include <utility>
#include "constants.hpp"
#include "helpers.hpp"
#include "spline.h"
#include "types.hpp"

/*
 * move in straight line at given speed
 * @param ego: our vehicle state
 * @param speed: target speed
 */
path_t move_straight(const car_t& ego, double speed);

/*
 * generates a spare path to follow in the specified lane
 * @param ego: our ego vehicles state
 * @param lane: current lane to follow
 * @param prev_path: left over previous path
 * @param map: waypoint map
 */
path_t move_in_lane(const car_t &ego, int lane, double speed, const path_t &prev_path, const map_t &map);

/*
 * checks if collision eminent in lane
 * @param lane: your lane
 * @param ego_s: s value for ego vehicle
 * @param end_path_s: s value at the end of last path
 * @param prev_path_size: path size from prev step
 * @param other_cars: vector of the other vehicles on the road
 */
bool check_collision_in_lane(int lane, double ego_s, double end_path_s, int prev_path_size, const std::vector<car_t>& other_cars);

/*
 * checks if lane is collision free for switching
 * @param lane: your lane
 * @param ego_s: s value for ego vehicle
 * @param end_path_s: s value at the end of last path
 * @param prev_path_size: path size from prev step
 * @param other_cars: vector of the other vehicles on the road
 * @param safe_distance: +/- distance in lane that is considered safe
 */
bool check_lane_for_switch(int lane, double ego_s, double end_path_s, int prev_path_size, const std::vector<car_t>& other_cars, double safe_distance);

/*
 * transform path from relative to absolute coordinates
 * @param x: car x location
 * @param y: car y location
 * @param yaw: car yaw
 * @param path: path is transformed in place
 */
void transform_path_abs(double x, double y, double yaw, path_t& path);

/*
 * transform path from relative to absolute coordinates
 * @param ref_x: car x location
 * @param ref_y: car y location
 * @param ref_yaw: car yaw
 * @param point: point is transformed in place
 */
void transform_point_abs(double ref_x, double ref_y, double ref_yaw, std::pair<double, double>& point);

/*
 * transform path from absolute to relative coordinates
 * @param x: relative x
 * @param y: relative y
 * @param yaw: relative angle
 * @param path: path is transformed in place
 */
void transform_path_rel(double x, double y, double yaw, path_t& path);

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
 * @return s, d as pair
 */
std::pair<double, double> get_frenet(double x, double y, double theta, const map_t& map);

/*
 * transform frenet s, d to cartesian x, y coods
 * @param s, d: frenet coods
 * @param map: waypoint map
 * @return x, y as pair
 */
std::pair<double, double> get_cartesian(double s, double d, const map_t& map);

/*
 * returns frenet d value for a particular lane
 */
double get_d_from_lane(int lane);