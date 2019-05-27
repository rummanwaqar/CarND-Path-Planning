#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <uWS/uWS.h>
#include "json.hpp"

#include "types.hpp"

/*
 * callback function definition
 * takes car localization, prev path and other car data
 * returns new path (path_t)
 */
typedef std::function< path_t(car_t my_car, path_t prev_path, double end_path_s,
   double end_path_d, std::vector<car_t> other_cars) > ProcessCb;

/*
 * Interface to simulator
 */
class SimIO {
public:
  /*
   * Constructor
   * Creates uWebSocket object and defines all event handlers
   * @param port - port number for simulator uWebSocket
   * @param cb callback for processing function
   */
  SimIO(int port, ProcessCb cb);

  /*
   * Destructor
   */
  ~SimIO() = default;

  /*
   * Initializes connection to simulator and blocks it until simulator is closed.
   * event handling for simulator
   */
  void run();

private:
  /*
   * Checks if the SocketIO event has JSON data.
   * If there is data the JSON object in string format will be returned,
   * else the empty string "" will be returned.
   */
  std::string hasData(std::string s);

  // uWS object
  uWS::Hub h_;

  // connection port
  int port_;

  // processing callback
  ProcessCb callbackFunc_;
};
