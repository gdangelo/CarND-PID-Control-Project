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

void run_car(Twiddle tw, PID &pid, double cte, uWS::WebSocket<uWS::SERVER> ws) {
  // Predict steering angle from errors
  pid.UpdateError(cte);
  double steer_value = -pid.TotalError();

  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = 0.3;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";

  // Log info: only in running mode
  if (!tw.is_used) {
    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
    std::cout << msg << std::endl;
  }

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
  // Initialize the pid variable.
  double Kp = atof(argv[1]);
  double Ki = atof(argv[2]);
  double Kd = atof(argv[3]);
  pid.Init(Kp, Ki, Kd);

  // Initialize the twiddle variable.
  Twiddle tw(atoi(argv[4]), pid);

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
          if (tw.is_used) {

            // Keep the car going
            tw.dist_count += 1;
            // Update error
            tw.error += cte*cte;
            tw.avg_error = tw.error / (tw.dist_count*tw.dist_count);

            // Stop current simulation loop when distance is reached
            // or the car is going off the road
            if (tw.dist_count > 50 && (tw.DistanceReached() || std::fabs(cte) >= 3.0)) {

              tw.PrintStepState(pid);

              // Initialize twiddle (first run)
              if (!tw.is_initialized) {
                tw.Init(pid);
                std::cout << "Initialization is done!" << std::endl;
              }
              // Handle PID parameter changes
              else {
                if (tw.avg_error < tw.best_error) {
                  // New best error found
                  tw.UpdateBestError();
                  // Change parameter index
                  tw.ChangePIDIndex();
                }
                else {
                  // Try going backward if forward did not succeed
                  if (tw.dp[tw.param_index].direction == DIRECTION::FORWARD) {
                    tw.GoBackward(pid);
                  }
                  // In case of both failed (fwd and bwd), reset PID parameter,
                  // decrease the update parameter dp, and switch PID parameter
                  else {
                    tw.ResetPIDParameter(pid);
                    tw.ChangePIDIndex();
                  }
                }
              }

              if (tw.dp[tw.param_index].direction == DIRECTION::FORWARD) {
                // Log info
                if (tw.param_index == 0) {
                  tw.PrintIterationState(pid);
                }
                tw.UpdatePIDParameter(pid);
              }

              // Reset distance and current run error
              tw.dist_count = 0;
              tw.error = 0;
              tw.avg_error = 0;

              // Reset the simulator
              reset_simulator(ws);
            }
          }

          run_car(tw, pid, cte, ws);
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

  h.onConnection([&h, &tw](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    if (tw.it == 0) {
      std::cout << "Connected!!!\n" << std::endl;
    }
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
