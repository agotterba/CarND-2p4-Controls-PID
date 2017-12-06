#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "TWIDDLE.h"
#include <math.h>
#include <time.h>
#include <chrono>

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

int main()
{
  uWS::Hub h;

  PID pid_steering;
  PID pid_st33ring;
  PID pid_throttle;
  TWIDDLE twiddle;
  bool use_twiddle;
  // TODO: Initialize the pid variable.
  //pid_steering.Init(0.2,0.05,0.0); //initial guess; will update later
  //pid_throttle.Init(0.1,0.0,0.0);
  //pid_steering.Init(1.200e-1,0.000e-0,0.000e-0); //trial and error to just get around the track (succeeded once, then never again) speed is 20
  //pid_throttle.Init(1.000e-1,0.000e-0,0.000e-0);
  //Ku = 0.12; period = 117 (counting steps in log file)
  //pid_steering.Init(7.200e-2,1.231e-3,1.053e-0); //Ziegler-Nichols method from GG paper
  //pid_throttle.Init(1.000e-1,0.000e-0,0.000e-0);
  //pid_steering.Init(1.000e-1,1.000e-3,5.000e-1); //hand modification to get around track (still failed)
  //pid_throttle.Init(1.000e-1,0.000e-0,0.000e-0);
  //pid_steering.Init(7.000e-2,1.200e-3,1.050e-0); //Ziegler-Nichols with new speed (failed)
  //pid_throttle.Init(3.000e-1,0.000e-0,0.000e-0);
  //pid_steering.Init(1.100e-1,5.000e-4,0.000e-0); //from previous good run
  //pid_throttle.Init(3.000e-1,0.000e-0,0.000e-0);
  //twiddle.init(1.100e-1,5.000e-4,0.000e-0,15);

  //initial values from manual tuning
  // double init_steer_kp = 1.100e-1;
  // double init_steer_kd = 5.000e-4;
  // double init_steer_ki = 0.000e-0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;
  // double init_target_speed  = 15.0;

  //better values from 1st round of twiddle debugging; cum error of 38 over 200 steps
  // double init_steer_kp = 2.199e-1;
  // double init_steer_kd = 8.329e-4;
  // double init_steer_ki = 8.658e-2;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;
  // double init_target_speed  = 15.0;

  //better values from 2nd round of twiddle debugging: cum_err of 1053 over 3k steps
  // double init_steer_kp = 2.951e-1;
  // double init_steer_kd = 9.949e-4;
  // double init_steer_ki = 1.326e-1;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;
  // double init_target_speed  = 15.0;

  //reset to manual values due to oscillations in straight, even though tight turns looked good
  // double init_steer_kp = 1.100e-1;
  // double init_steer_kd = 5.000e-4;
  // double init_steer_ki = 1.000e-1;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;
  // double init_target_speed  = 15.0;

  //fixed twiddle's oscillations; values after epoch 21:
  // double init_steer_kp = 1.18067e-1;
  // double init_steer_kd = 5.16667e-4;
  // double init_steer_ki = 1.03333e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  // simulation crashed during epoch 18, after raising speed; check last good run
  // double init_steer_kp = 1.15241e-1;
  // double init_steer_kd = 5.21920e-4;
  // double init_steer_ki = 1.03817e-1;
  // double init_target_speed  = 18.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  //reset to good run at 15mph
  // double init_steer_kp = 1.18067e-1;
  // double init_steer_kd = 5.16667e-4;
  // double init_steer_ki = 1.03333e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  // best values after 46 epochs at 15mph (never got max cde below 2.5m)
  // double init_steer_kp = 1.15488e-1;
  // double init_steer_kd = 5.19759e-4;
  // double init_steer_ki = 1.03939e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

// decreased rate of integral unwinding, and got these new values
  // double init_steer_kp = 1.15423e-1;
  // double init_steer_kd = 5.20259e-4;
  // double init_steer_ki = 1.03938e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

// added cube term
  // double init_steer_kc = 5.00000e-2; 
  // double init_steer_kp = 1.15423e-1;
  // double init_steer_kd = 5.20259e-4;
  // double init_steer_ki = 1.03938e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kc = 0.000e-0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  // after 34 epochs (speed got to 16, but leave at 15 here); also, switch to two pid instead of an extra cube term
  // double init_steer_cp = 5.01126e-2; 
  // double init_steer_cd = 2.00000e-4; 
  // double init_steer_ci = 5.00000e-2; 
  // double init_steer_kp = 1.15054e-1;
  // double init_steer_kd = 5.20336e-4;
  // double init_steer_ki = 1.03938e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  //after 20 epochs (speed got to 16 in twiddle, but leave at 15 here);
  // double init_steer_cp = 5.41126e-2; 
  // double init_steer_cd = 1.87200e-4; 
  // double init_steer_ci = 4.11600e-2; 
  // double init_steer_kp = 1.11777e-1;
  // double init_steer_kd = 4.88336e-4;
  // double init_steer_ki = 1.03938e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  //found horrible typo/bug.  retrain with only 1 pid in effect; will add second later.  Revert to params before adding.
  // double init_steer_cp = 0.00000e-2;
  // double init_steer_cd = 0.00000e-4; 
  // double init_steer_ci = 0.00000e-2; 
  // double init_steer_kp = 1.15423e-1;
  // double init_steer_kd = 5.20259e-4;
  // double init_steer_ki = 5.00000e-2;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 1.000e-2;
  // double init_speed_ki = 1.000e-2;

  //those didn't make it around the track, so back to the full manual settings :(
  // double init_steer_cp = 0.00000e-2;
  // double init_steer_cd = 0.00000e-4; 
  // double init_steer_ci = 0.00000e-2; 
  // double init_steer_kp = 1.20000e-1;
  // double init_steer_kd = 5.00000e-5;
  // double init_steer_ki = 1.00000e-4;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 1.000e-2;
  // double init_speed_ki = 1.000e-2;

  //got good values after epoch 35; will add cube terms back in.  
  // double init_steer_cp =  5.00000e-2;
  // double init_steer_cd = -3.00000e-5; 
  // double init_steer_ci =  1.80000e-4; 
  // double init_steer_kp =  1.20000e-1;
  // double init_steer_kd = -7.50000e-5;
  // double init_steer_ki =  5.25000e-4;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 1.000e-2;
  // double init_speed_ki = 1.000e-2;

  //got good values after epoch 22 with all params
  double init_steer_cp =  5.00024e-2;
  double init_steer_cd = -3.07115e-5; 
  double init_steer_ci =  1.85059e-4; 
  double init_steer_kp =  1.22560e-1;
  double init_steer_kd = -7.61943e-5;
  double init_steer_ki =  4.58138e-4;
  double init_target_speed  = 15.0;
  double init_speed_kp = 3.000e-1;
  double init_speed_kd = 1.000e-2;
  double init_speed_ki = 1.000e-2;

  //twiddled values from first attempt (before bug was fixed, for comparison)
  // double init_steer_cp = 5.41126e-2; 
  // double init_steer_cd = 1.87200e-4; 
  // double init_steer_ci = 4.11600e-2; 
  // double init_steer_kp = 1.11777e-1;
  // double init_steer_kd = 4.88336e-4;
  // double init_steer_ki = 1.03938e-1;
  // double init_target_speed  = 15.0;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;

  //manual tuning coefficents, for recording
  // double init_steer_cp = 0.00000e-2; 
  // double init_steer_cd = 0.00000e-4; 
  // double init_steer_ci = 0.00000e-2; 
  // double init_steer_kp = 1.100e-1;
  // double init_steer_kd = 5.000e-4;
  // double init_steer_ki = 1.000e-1;
  // double init_speed_kp = 3.000e-1;
  // double init_speed_kd = 0.000e-0;
  // double init_speed_ki = 0.000e-0;
  // double init_target_speed  = 15.0;

  
  pid_steering.Init("mSteering",init_steer_kp,init_steer_kd,init_steer_ki);
  pid_st33ring.Init("mSt33ring",init_steer_cp,init_steer_cd,init_steer_ci); 
  pid_throttle.Init("mThrottle",init_speed_kp,init_speed_kd,init_speed_ki);
  twiddle.init(init_steer_kp,init_steer_kd,init_steer_ki,
               init_steer_cp,init_steer_cd,init_steer_ci,
               init_speed_kp,init_speed_kd,init_speed_ki,
               init_target_speed);
  use_twiddle = false;

  auto start_time = std::chrono::system_clock::now();
  //h.onMessage([&pid_steering,&pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&start_time,&pid_steering,&pid_st33ring,&pid_throttle,&twiddle,&init_target_speed,&use_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());//not used
          double steering_value;
          double throttle_value;
          bool reset = false;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          double speed_error;
          if (use_twiddle){
            twiddle.update(cte,speed);
            reset = twiddle.checkReset(); //separate return so that reset of sim is handled from here
            if(reset){
              std::cout<<"main received reset\n";
            }else{
              steering_value = twiddle.findSteering();
              throttle_value = twiddle.findThrottle();
              speed_error    = twiddle.get_speed_error();
            }
          }else{
            pid_steering.UpdateError(cte);
            pid_st33ring.UpdateError(cte*cte*cte);
            steering_value = pid_steering.TotalError() + pid_st33ring.TotalError();
            speed_error = speed - init_target_speed;
            pid_throttle.UpdateError(speed_error);
            throttle_value = pid_throttle.TotalError();
          }
          //double mytime = ((double)clock()) / ((double)CLOCKS_PER_SEC);
          auto cur_time = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsed_seconds = cur_time - start_time;
          // DEBUG
          std::cout << "Time: "<< elapsed_seconds.count() <<" CTE: " << cte << " Steering angle: " << steering_value * 25.0 << " received angle: " << angle << " speed_error: " << speed_error << " Throttle Value: " << throttle_value << std::endl;
          json msgJson;
          if (reset){
            std::cout <<"resetting simulation\n";
            std::string reset_msg = "42[\"reset\",{}]";
            std::cout << reset_msg << std::endl;
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

          }else{
            msgJson["steering_angle"] = steering_value;
            //msgJson["throttle"] = 0.3;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
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
    std::cout << "Connected!!!" << std::endl;
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
