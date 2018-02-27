#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void run_car(PID pid, double cte, uWS::WebSocket<uWS::SERVER> ws) {
  pid.UpdateError(cte);
  double steer_value = -pid.TotalError();

  // DEBUG
  //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = 0.3;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  //std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void reset_simulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  Twiddle tw(atoi(argv[1]), 0.0);

  // Initialize the pid variable.
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
  pid.Init(Kp, Ki, Kd);

  h.onMessage([&pid, &tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // Use parameters optimization (twiddle)
          if(tw.is_used) {

            // Keep the car going
            tw.dist_count += 1;
            // Update error
            tw.error += cte*cte;

            if(tw.DistanceReached()) {

              // STEP 1: Initialize twiddle (first run)
              if(!tw.is_initialized) {
                tw.Init(pid);
                std::cout << "Initialization is done." << std::endl;
                std::cout << "Best error: " << tw.best_error << std::endl;
                std::cout << "Kp optimization begins..." << std::endl;
              }
              // STEP 2: Update PID parameters
              else {
                // New best error found
                if(tw.error < tw.best_error) {
                  tw.best_error = tw.error;
                  tw.dp[tw.param_index].value *= 1.1;
                  tw.dp[tw.param_index].direction = DIRECTION::FORWARD;

                  std::cout << "Best error: " << tw.best_error << std::endl;

                  std::cout << "Best PID params: "
                            << pid.Kp << "(Kp), "
                            << pid.Ki << "(Ki), "
                            << pid.Kd << "(Kd)"
                            << endl;

                  // Change parameter index and update value
                  tw.ChangePIDIndex();
                  tw.UpdatePIDValue(pid);
                }
                else {
                  // Try going backward if forward did not succeed
                  if(tw.dp[tw.param_index].direction == DIRECTION::FORWARD) {
                    std::cout << "Go backward..." << std::endl;
                    tw.GoBackward(pid);
                  }
                  // In case of both failed (fwd and bwd), reset PID parameter K
                  // and decrease the update parameter dp
                  else {
                    switch(tw.param_index) {
                      case 0:
                        std::cout << "Reset parameter Kp." << std::endl;
                        pid.Kp += tw.dp[tw.param_index].value;
                        break;
                      case 1:
                        std::cout << "Reset parameter Ki." << std::endl;
                        pid.Ki += tw.dp[tw.param_index].value;
                        break;
                      case 2:
                        std::cout << "Reset parameter Kd." << std::endl;
                        pid.Kd += tw.dp[tw.param_index].value;
                        break;
                    }
                    tw.dp[tw.param_index].value *= 0.9;
                    tw.dp[tw.param_index].direction = DIRECTION::FORWARD;

                    // Change parameter index and update value
                    tw.ChangePIDIndex();
                    tw.UpdatePIDValue(pid);
                  }
                }
              }

              // Reset distance and current run error
              tw.dist_count = 0;
              tw.error = 0;

              // Reset the simulator
              reset_simulator(ws);
            }
          }

          run_car(pid, cte, ws);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!\n" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
