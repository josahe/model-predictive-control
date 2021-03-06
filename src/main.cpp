#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <stdlib.h>
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

// Keeps a record of the real latency of the solver and simulator
std::chrono::time_point<std::chrono::system_clock> then;
double calc_latency(std::chrono::time_point<std::chrono::system_clock> then) {
  auto now = std::chrono::system_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(now - then);
  return dur.count() / 1000.;
}

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
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Global Kinematic Model
// Returns updated state using the model, by predicting the next state after a given latency, dt
void globalKinematic(double &x, double &y, double &p, double &v, double d, double a, double dt) {
  double new_v = v + a*dt;
  double new_p = p + (v/Lf)*(-d)*dt;
  double new_x = x + v*cos(p)*dt;
  double new_y = y + v*sin(p)*dt;

  x=new_x;
  y=new_y;
  v=new_v;
  p=new_p;
}

int main(int argc, char **argv) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // initiate for first latency calculation
  then = std::chrono::system_clock::now();

  // set weights for cost functions
  std::vector<double> weights = {2, 10, 5, 3000, 100, 500, 100};

  // Override weights if passed in on command line
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      weights[i-1] = atof(argv[i]);
    }
  }

  std::cout << "Weights are: ";
  for (unsigned int i = 0; i < weights.size(); ++i) {
    std::cout << weights[i] << " ";
  }
  std::cout << std::endl;

  // pass weights to model
  mpc.pass_args(weights);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // NOTE j[1] is the data JSON object

          // Waypoints. The points in the middle of the track corresponding to
          // the ideal trajectory of the car
          vector<double> ptsx = j[1]["ptsx"]; // waypoints x
          vector<double> ptsy = j[1]["ptsy"]; // waypoints y

          // Current vehicle state.
          double px = j[1]["x"]; // current x
          double py = j[1]["y"]; // current y
          double psi = j[1]["psi"]; // current heading
          double v = j[1]["speed"]; // current velocity TODO convert to m/s
          double d = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          // Simulate dynamics and account for latency
          globalKinematic(px, py, psi, v, d, a, calc_latency(then));

          then = std::chrono::system_clock::now();

          // Transform waypoints from map to vehicle coordinates
          Eigen::VectorXd ptsx_transform(ptsx.size());
          Eigen::VectorXd ptsy_transform(ptsy.size());
          for (unsigned int i = 0; i < ptsx.size(); ++i) {
            // shift the origin of waypoint relative to car
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            // rotate waypoint relative to car heading
            ptsx_transform[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_transform[i] = x * sin(-psi) + y * cos(-psi);
          }

          // Fit a third order polynomial to the waypoints
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // Calculate cross track error by evaluating the polynomial at x=0
          double cte = polyeval(coeffs, 0);

          // Calculate error psi (the difference between car heading and
          // optimal heading)
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state_vector(6);
          //state_vector << px, py, psi, v, cte, epsi;
          state_vector << 0, 0, 0, v, cte, epsi;

          actuation_vars vars = mpc.Solve(state_vector, coeffs);

          // Set steering angle and throttle solved by MPC
          double steer_value = vars.d;
          double throttle_value = vars.a;

          /* -= Yellow Line =-
            Display the waypoints/reference line using the polynomial
          */
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int num_points = 25;
          double unit = 2.5;
          for (int i = 0; i < num_points; ++i) {
            next_x_vals.push_back(i*unit);
            next_y_vals.push_back(polyeval(coeffs, i*unit));
          }

          /* -= Green Line =-
            Display the MPC predicted trajectory
          */
          vector<double> mpc_x_vals = vars.x_vals;
          vector<double> mpc_y_vals = vars.y_vals;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // NOTE: The heading update equation uses positive delta to infer a
          // counter-clockwise (left) rotation. However, in the simulator, a
          // positive value implies a right turn, so multiply by -1
          json msgJson;
          msgJson["steering_angle"] = -1 * (steer_value / deg2rad(25));
          msgJson["throttle"] = throttle_value;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
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
          // TODO add latency to the model
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
  // program doesn't compile :-(
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
