#include "io.hpp"

SimIO::SimIO(int port, ProcessCb cb) : port_(port), callbackFunc_(cb) {
  /*
   * Register event handlers for uWS
   */
  h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string s = hasData(std::string(data));
      if(s != "") { // data available
        // parse json
        auto j = nlohmann::json::parse(s);
        std::string event = j[0].get<std::string>();

        if(event == "telemetry") {
          // Main car's localization Data
          car_t sdc;
          sdc.x = j[1]["x"];
          sdc.y = j[1]["y"];
          sdc.s = j[1]["s"];
          sdc.d = j[1]["d"];
          sdc.yaw = j[1]["yaw"];
          sdc.speed = j[1]["speed"];

          // Previous path data given to the Planner (processed points are removed)
          path_t prev_path{j[1]["previous_path_x"], j[1]["previous_path_y"]};

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // list of all other cars on the same side of the road.
          std::vector<car_t> cars;
          for(auto const& car : j[1]["sensor_fusion"]) {
            car_t temp_car;
            temp_car.id = car[0];
            temp_car.x = car[1];
            temp_car.y = car[2];
            temp_car.vel_x = car[3];
            temp_car.vel_y = car[4];
            temp_car.s = car[5];
            temp_car.d = car[6];
            cars.push_back(temp_car);
          }

          // define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          path_t new_path = callbackFunc_(std::move(sdc), std::move(prev_path),
            end_path_s, end_path_d, std::move(cars));

          // generate new json message with path and send
          nlohmann::json msgJson;
          msgJson["next_x"] = new_path.x;
          msgJson["next_y"] = new_path.y;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h_.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h_.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
}

void SimIO::run() {
  // listen and wait for connection
  if (h_.listen(port_)) {
    std::cout << "Listening to port " << port_ << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return;
  }
  // endless loop until application exists
  h_.run();
}

std::string SimIO::hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
