#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];

          double time_lapse = 0.1;
          double px_next = px + (v * cos(psi) * time_lapse);
          double py_next = py + (v * sin(psi) * time_lapse);
          double psi_next = psi - ((v * delta * time_lapse)/2.67);
          // double psi_next = psi;
          double v_next = v + (acceleration * time_lapse);

          // TODO 1 - Shift the coordinates of ptsx and ptsy to origin of car
          // TODO 2 - Rotate the coordinates of ptsx and ptsy to bring them w.r.t. psi of car
          // TODO 3 - Polyfit accepts Eigen vector and ptsx and ptsy are std::vectors. Make this 
          //          conversion
          // Convert global coordinates to vehicle coordinates so that formula of cte and epsi is easy and involves less calculation
          double xdiff = 0;
          double ydiff = 0;
          Eigen::VectorXd ptsx_vehicle(ptsx.size());
          ptsx_vehicle.fill(0.0);
          Eigen::VectorXd ptsy_vehicle(ptsy.size());
          ptsy_vehicle.fill(0.0);
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            xdiff = ptsx[i] - px_next;
            ydiff = ptsy[i] - py_next;

            // xdiff = ptsx[i] - px;
            // ydiff = ptsy[i] - py;
            
            ptsx_vehicle[i] = xdiff * cos(-psi_next) - ydiff * sin(-psi_next);
            ptsy_vehicle[i] = xdiff * sin(-psi_next) + ydiff * cos(-psi_next);

            // ptsx_vehicle[i] = xdiff * cos(-psi) - ydiff * sin(-psi);
            // ptsy_vehicle[i] = xdiff * sin(-psi) + ydiff * cos(-psi);
            
            // std::cout<<"Transformed X is: "<<ptsx_vehicle[i]<<std::endl;
            // std::cout<<"Transformed Y is: "<<ptsy_vehicle[i]<<std::endl;
          }
          auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);
         
          //TODO 4 - Calculate cte and epsi. cte is the horizontal line
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          //TODO 5 - Create the state
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v_next, cte, epsi;
          // state << 0, 0, 0, v, cte, epsi;
          
          //TODO 6 - Set the steering angle to delta and throttle to a for current time step
          //          solved by MPC
          std::vector<double> solution;
          solution = mpc.Solve(state, coeffs);
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = solution[0]/deg2rad(25);
          double throttle_value = solution[1];
          // double steer_value;
          // double throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0 * steer_value;
          msgJson["throttle"] = throttle_value;
          
          //TODO 7 - Use polyfit to plot green line using vars solved by MPC
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (unsigned int i = 2; i < solution.size(); i++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(solution[i]);
            } else {
              mpc_y_vals.push_back(solution[i]);
            }
          }
          
          

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // TODO 8 - Use polyfit to plot yellow line using way points
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for (int i = 0; i < ptsx_vehicle.size(); i++) {
            next_x_vals.push_back(ptsx_vehicle[i]);
            next_y_vals.push_back(ptsy_vehicle[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          // this_thread::sleep_for(chrono::milliseconds(0));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
