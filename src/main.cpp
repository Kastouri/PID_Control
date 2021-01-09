#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

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
void resetSim(uWS::WebSocket<uWS::SERVER> ws){   
 std::string reset_msg = "42[\"reset\",{}]";    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
int main() {
  uWS::Hub h;

  /**
   * Twiddle 
   */
  Twiddle twiddle; 

  // parameters vector
  twiddle.set_p({0.129228, 0.000114816, 1.96532});//{0.114601, 0.00018, 2.36});//{0.15, 0.0002 , 2.8});

  // twiddle vector
  const bool tune = false;
  vector<double> dp;
  if (tune) 
    dp = {twiddle.p[0] * 0.1, twiddle.p[1] * 0.1, twiddle.p[2] * 0.1};
  else 
    dp = {0 ,0 ,0};
  twiddle.set_dp(dp); //{twiddle.p[0] * 0.1, twiddle.p[1] * 0.1,twiddle.p[2] * 0.1};
  
  double best_error = 0;
  double error = 0;
  long long counter = 0;
  const int sim_n = 500;
  long long iteration = 0;
  int param_index = 0;
  bool tweaked_up = true;
  

  h.onMessage([ 
                &tweaked_up, &param_index, &error, &tune,
                &best_error, &counter, &iteration, &twiddle
                ]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double steer_value;
          double throttle = 0.3;
          

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          // calculate the total error
          steer_value = twiddle.control_out(cte);

          // clip range
          if (steer_value > 1.0) steer_value = 1.0;
          if (steer_value < -1.0) steer_value = -1.0;
          
          // adjust throttle: speeding up when steering is dangerous
          throttle -= abs(steer_value) * 0.5;

          // DEBUG
          /* std::cout << "Counter: "<< counter <<" CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;
          */
          // accumulate the error
          if (counter > 30.0) {
            error += cte * cte;
          }
          // increment counter
          counter++;
          
          if (counter % sim_n == 0) {  // start new simulation
            /**
             *  Initialisation:
             *  - Set the initial weights of the controller
             *  - Set the initial twiddle values
             */
            if (iteration == 0) {
              iteration++;
              // pid.Init(0.15, 0.002 , 3.0);
              best_error = error;
              // tweak controller for next step
              tweaked_up = true;
              twiddle.tweak(param_index, tweaked_up);
            } 
            else {
              if (tweaked_up == true) {
                if (error < best_error) {
                  iteration++;
                  //std::cout<< "Iteration "<< iteration <<" Best Accumulated Squared Error : " << best_error << std::endl;
                  best_error = error;
                  twiddle.adjust_dp(param_index, true);  // increment dp
                } else {
                  tweaked_up = false;
                  twiddle.tweak(param_index, tweaked_up);
                }
              }
              else {
                if (error < best_error) {
                  iteration++;
                  //std::cout<< "Iteration "<< iteration <<" Best Accumulated Squared Error : " << best_error << std::endl;
                  best_error = error;
                  twiddle.adjust_dp(param_index, true); // increment dp
                }
                else {
                  iteration++;
                  //std::cout<< "Iteration "<< iteration  <<" Best Accumulated Squared Error : " << best_error << std::endl;
                  tweaked_up = true;
                  twiddle.tweak(param_index, tweaked_up);
                  twiddle.adjust_dp(param_index, false);  // decrement dp
                }
              }
            }
            param_index = (param_index + 1) % 3;
            if (tune) std::cout<< "Iteration "<< iteration <<" Best Accumulated Squared Error : " << best_error << std::endl;
            if (tune) twiddle.print_params();      
            // restart simulation
            if (tune) resetSim(ws);
            //ws.close();
            // reset error for the next simulation
            error = 0.0;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage
  
  /*
  }
   counter = 0;
  error--;
  } */

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