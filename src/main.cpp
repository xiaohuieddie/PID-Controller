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

  PID pid_steer, pid_throttle;
  /**
   * TODO: Initialize the pid variable.
   */
  pid_throttle.Init(0.316731, 0.0000, 0.0226185);

  int step = 0;
  double total_cte = 0.0;
  double best_error = 9999.9;
  int iterator = 0;
  bool twiddle = false;
  bool first_run = true;     // first run inside the Twiddle algorithm to get the error of the new PID value
  bool second_run = true;    // second run ...
  int sub_move = 0;          // It is used to illustrate whether one iterator is updated. Once it is done, move to next iterator (Kp, Ki, Kd) in Twiddle algorithm
  double dp[3] = {.01, .0001, .1};
  double p[3] = {0.05, 0.0001, 1.5};
  double best_p[3] = {p[0],p[1],p[2]};
  
  if(twiddle == true) {
    pid_steer.Init(p[0],p[1],p[2]);
  }else {
    //pid_steer.Init(0.06, 0.00031, 1.29);
    pid_steer.Init(0.2, 0.00031, 3.0);
  }

  h.onMessage([&sub_move, &first_run, &second_run, &iterator, &best_p, &dp, &p, &best_error, &twiddle, &total_cte, &step, &pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double throttle_value = 0.25;
          json msgJson;
          //
          double error;
          double max_step = 600;
          // ----------------------------------------------------------------
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (twiddle == true){
            total_cte += pow(cte, 2);
            // initialize the PID value for the first step
            if (step == 0){
              pid_steer.Init(p[0], p[1], p[2]);
            }
            pid_steer.UpdateError(cte);
            steer_value = pid_steer.TotalError();
            step += 1;   // Update step value when each cte value comes in
            
            // Apply Twiddle algorithm when step is lager than 600
            if (step >= max_step){
              if (first_run == true){
                p[iterator] += dp[iterator];
                first_run = false;
              } else {
                error = total_cte / max_step;
                if (error < best_error && second_run == true){
                  best_error = error;
                  best_p[0] = p[0];
                  best_p[1] = p[1];
                  best_p[2] = p[2];
                  dp[iterator] *= 1.1;
                  sub_move += 1;
                } else {
                  if (second_run == true){
                    p[iterator] -= 2 * dp[iterator];
                    second_run = false;
                  } else{
                    if (error < best_error){
                      best_error = error;
                      best_p[0] = p[0];
                      best_p[1] = p[1];
                      best_p[2] = p[2];
                      dp[iterator] *= 1.1; 
                      sub_move += 1;
                    } else {
                      p[iterator] += dp[iterator];
                      dp[iterator] *= 0.9;
                      sub_move += 1;
                    }
                  }
                  
                }
              }
              
              // Move to this iterator once it is done
              if(sub_move > 0) {
                iterator += 1;
                first_run = true;
                second_run = true;
                sub_move = 0;
              }
              
              // Reset iterator to 0 (Kp)
              if(iterator == 3){
                iterator = 0;
              }
              
              // Reset the total_cte/step to zero after reset the simulator;
              total_cte = 0;
              step = 0;
              
              // Evaluate if sum of dp[] is less than the tolerance
              double sumdp = dp[0]+dp[1]+dp[2];
              if(sumdp < 0.001) {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
              } else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            } else { // don't apply twiddle algorithm for the first 600 steps
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
            
          } else { // if twiddle is set to False
            pid_steer.UpdateError(cte);
            //pid_throttle.UpdateError(cte);
            steer_value = pid_steer.TotalError();
            //throttle_value = 0.3 + pid_throttle.TotalError();
            // -----------------------------------------------------------
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);   
          }

          
          
          
          
          
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