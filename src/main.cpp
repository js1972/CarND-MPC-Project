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

/**
 * === MAIN =============================== 
 */
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          vector<double> plot_x_vals;
          vector<double> plot_y_vals;

          //// Convert observation to world/map coordinates
          //double obs_x, obs_y;
          //obs_x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
          //obs_y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);

          // Convert x, y waypoints (in global coords) to car coordinates
          for (int i=0; i<ptsx.size(); i++) {
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;
            ptsx[i] = x_diff * cos(-psi) - y_diff * sin(-psi);
            ptsy[i] = x_diff * sin(-psi) + y_diff * cos(-psi);

            //double x = ptsx[i];
            //double y = ptsy[i];
            //ptsx[i] = x * cos(psi) - y * sin(psi) + px;
            //ptsy[i] = x * sin(psi) + y * cos(psi) + py;

            plot_x_vals.push_back(ptsx[i]);
            plot_y_vals.push_back(ptsy[i]);
          }

          //psi = psi - pi();

          Eigen::VectorXd ptsxv = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsyv = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          // Fit a 3rd order polynomial to the given (transformed) way points and
          // calculate the cross track error and psi error values (determined from
          // atan(derivative of the polynomial)).
          auto coeffs = polyfit(ptsxv, ptsyv, 3);
          double cte = polyeval(coeffs, px) - py;
          //double epsi = -atan(coeffs[1]);
          double epsi = psi - atan(coeffs[1]+coeffs[2]*2*px+coeffs[3]*3*px*px);

          // Setup the state
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          std::cout << "STATE: " << px << "\t" << py << "\t" << psi << "\t" << v << "\t" << cte << "\t" << epsi << std::endl;

          auto vars = mpc.Solve(state, coeffs);
          
          
          // Generate a range of x, y values from our fitted polynomial for plotting (N=25)
          //vector<double> plot_x_vals(25);
          //vector<double> plot_y_vals(25);
          //int inc_x = 0;
          //int inc_y = 0;
          //std::generate(plot_x_vals.begin(), plot_x_vals.end(), [&]{ inc_x++; return 2.5*inc_x; });
          //std::generate(plot_y_vals.begin(), plot_y_vals.end(), [&]{ inc_y++; return polyeval(coeffs, 2.5*inc_y); });

          //std::cout << "plot_x_vals:" << std::endl;
          //for (int i=0; i< plot_x_vals.size(); i++) {
          //  std::cout << plot_x_vals[i] << "\t";
          //}
          //std::cout << std::endl;

          std::cout << "VARS:" << std::endl;
          for (int i=0; i<vars.size(); i++) {
            std::cout << vars[i] << "\t";
          }
          std::cout << std::endl;

          double steer_value = -vars[0];
          double throttle_value = vars[1];

          // Generate a range of x, y values from our mpc generated (best) points for plotting.
          // Note: the vars structure is made up of the steering value, throttle value, then
          // repeating pairs of x,y coords.
          vector<double> plot_mpc_x_vals;
          vector<double> plot_mpc_y_vals;
          //for (int i=2; i<vars.size(); i++) {
          for (int i=2; i<8; i++) {
            if (i%2 == 0) {
              plot_mpc_x_vals.push_back(vars[i]);
              std::cout << "MPC X: " << vars[i] << std::endl;
            } else {
              plot_mpc_y_vals.push_back(vars[i]);
              std::cout << "MPC Y: " << vars[i] << std::endl;
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["next_x"] = plot_x_vals;
          msgJson["next_y"] = plot_y_vals;

          msgJson["mpc_x"] = plot_mpc_x_vals;
          msgJson["mpc_y"] = plot_mpc_y_vals;

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
