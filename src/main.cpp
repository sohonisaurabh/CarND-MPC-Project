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

//Global variable to switch on and off the controller.
bool start_controller = true;

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

int main(int argC, char** argV) {
  uWS::Hub h;

  //Conditional check for running the controller solver or not
  if (argC > 1) {
    std::string controller_config = argV[1];
    if (controller_config.compare("controller-off") == 0) {
      start_controller = false;
    }
  }
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

          //Incorporate latency into the model. time_lapse is 100ms or 0.1sec.
          double time_lapse = 0.1;
          std::vector<double> current_mea = {px, py, psi, v};
          std::vector<double> current_actuator_from_sim = {acceleration, delta};

          //Predict state for situation after latency
          std::vector<double> next_pred = mpc.PredictNextState(current_mea, current_actuator_from_sim, time_lapse);

          //1. - Shift the coordinates of ptsx and ptsy to origin of car
          //2. - Rotate the coordinates of ptsx and ptsy to bring them w.r.t. psi of car
          //3. - Polyfit accepts Eigen vector and ptsx and ptsy are std::vectors. Make this 
          //          conversion
          //4. Convert global coordinates to vehicle coordinates so that formula of cte and epsi is easy and involves less calculation
          double px_next = next_pred[0];
          double py_next = next_pred[1];
          double psi_next = next_pred[2];
          double v_next = next_pred[3];
          double xdiff = 0;
          double ydiff = 0;
          Eigen::VectorXd ptsx_vehicle(ptsx.size());
          ptsx_vehicle.fill(0.0);
          Eigen::VectorXd ptsy_vehicle(ptsy.size());
          ptsy_vehicle.fill(0.0);
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            xdiff = ptsx[i] - px_next;
            ydiff = ptsy[i] - py_next;

            ptsx_vehicle[i] = xdiff * cos(-psi_next) - ydiff * sin(-psi_next);
            ptsy_vehicle[i] = xdiff * sin(-psi_next) + ydiff * cos(-psi_next);
          }
          auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);
         
          //Calculate cte and epsi. cte is the horizontal line
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          //Create the state vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v_next, cte, epsi;
          
          //Placeholder for solution returned by optimizer
          std::vector<double> solution;
          double steer_value;
          double throttle_value;
          
          //Call the solver and set the steering angle to delta and throttle to a for current 
          //time step solved by MPC
          if (start_controller) {
            solution = mpc.Solve(state, coeffs);
            //Multiplying steering angle by -1 as the implementation of positive, negative
            //angles and right, left turn in simulator is reversed in comparison to co-ordinate
            //system in vehicle's plane
            steer_value = -1.0 * solution[0]/deg2rad(25);
            throttle_value = solution[1];
          }

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          //Use polyfit to plot green line using vars solved by MPC
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

          //Use polyfit to plot yellow line using way points
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
