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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // Position of the simulator waypoints (j[1] is the data JSON object)
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          // Car position, yaw angle and speed
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v = v * 0.44704; // conversion from mph to m/s
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Transform waypoints
          for (unsigned int i = 0; i < ptsx.size(); ++i) {
            // Calculate delta between waypoints
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            // Calculate transformed x and y waypoints
            ptsx[i] =  delta_x * cos(-psi) - delta_y * sin(-psi);
            ptsy[i] =  delta_x * sin(-psi) + delta_y * cos(-psi);
          }

          // Conversion to Eigen::VectorXd
          Eigen::VectorXd x_vals =
            Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd y_vals =
            Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

          // Fit a 3rd order polynomial to the reference lane line
          auto coeffs = polyfit(x_vals, y_vals, 3);

          // Calculate epsi (error of vehicle's yaw angle compared to track)
          double epsi = -atan(coeffs[1]);

          // Calculate cross-track-error (deviation from lane center)
          double cte = polyeval(coeffs, 0)*cos(epsi);

          // Vehicle and simulator variables for prediction and latency compensation
          const double Lf = 2.67;
          const double dt = 0.1;

          // Predict state after latency
          double p_px =   0.0 + v * dt; // Psi is modeled as zero during prediction
          double p_py =   0.0;          // with Psi modeled as zero, no lateral change
          double p_psi =  0.0 + v * -delta / Lf * dt;
          double p_v =    v + a * dt;
          double p_cte =  cte + v * sin(epsi) * dt;
          double p_epsi = p_psi + epsi;

          // Create current state vector including latency prediction
          Eigen::VectorXd curr_state(6);
          curr_state << p_px, p_py, p_psi, p_v, p_cte, p_epsi;

          // Calculate steering angle and throttle using MPC.
          // Both are in between [-1, 1].
          auto MPC_solution = mpc.Solve(curr_state, coeffs);

          // Extract steering angle and throttle value
          double steer_value = MPC_solution[0]; //
          double throttle_value = MPC_solution[1];

          // Display the line the MPC should follow in simulator (YELLOW LINE)
          vector<double> next_x_vals(30);
          vector<double> next_y_vals(30);
          for (int i = 0; i < 30; ++i) {
            next_x_vals[i] = 3 * i;
            next_y_vals[i] = polyeval(coeffs, next_x_vals[i]);
          }

          // Display the MPC predicted trajectory (GREEN LINE)
          unsigned int n_mpc_vals = (MPC_solution.size()-2) * 0.5;
          vector<double> mpc_x_vals(n_mpc_vals);  // Pre-allocate variable with proper size
          vector<double> mpc_y_vals(n_mpc_vals);
          for (unsigned int i = 2; i < MPC_solution.size()-1; i+=2) {
            // Extract x,y pairs: x has even, y has odd index
            mpc_x_vals[(i-2)*0.5] = MPC_solution[i];
            mpc_y_vals[(i-2)*0.5] = MPC_solution[i+1];
          }

          // Create response message for simulator
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          json msgJson;
          msgJson["steering_angle"] = steer_value / (Lf * deg2rad(25.));
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
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
