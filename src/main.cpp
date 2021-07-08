#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // proportional coefficient Kp_ influences amplitude of the current resulting value
  // high propotional coefficients may result in bigger oscilations around the target value
  //
  // differential coefficient Kd_influences how fast the resulting value converges to the target value
  // lower differential coefficients may result in more oscillations around the target value
  //
  // integral coefficient Ki_ helps to compensate constant shift of the resulting value from the target value
  //
  // coefficients here are tuned experimentally:
  // Kp_ is selected to be not very high to achieve smooth control actions and avoid too sharp steering
  // Kd_ is selected pretty high to minimize number of oscillations and faster converge to the tagret values
  // Ki_ is selected to slightly improve shift of the trajectory and keep the car within the lane through the whole track
  
  PID pid_steering;
  pid_steering.Init(0.1, 0.003, 5.0);
  
  PID pid_throttle;
  pid_throttle.Init(0.1, 0.003, 5.0);

  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([&pid_steering, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid_steering.UpdateError(cte);
          double steer_value = pid_steering.TotalError();
          
          // throttle is controlled with a separate PID
          // 0.3 is a maximum throttle
          // exp(-abs) converts throttle error into a factor between 0 and 1: lower the error, closer the factor is to 1
          // as a result, max throttle is scaled using the factor
          // 0.05 is a braking threshold that shifts resulting throttle maintaining the scale
          // if the error is too high and the factor is less than the threshold, then negative throttle is calculated to reduce the speed faster
          pid_throttle.UpdateError(cte);
          double throttle = 0.3 * (exp(-abs(pid_throttle.TotalError())) - 0.05);
          
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}