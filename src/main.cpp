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
  int n = coeffs.size();
  double result = coeffs[n-1];
  for (int i = n-2; i >= 0; i--) {
    result *= x; 
    result += coeffs[i];
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals,
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

void test(){
  string sdata = "42[\"telemetry\",{\"ptsx\":[-32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717],\"ptsy\":[113.361,105.941,92.88499,78.73102,65.34102,50.57938],\"psi_unity\":4.120315,\"psi\":3.733667,\"x\":-40.62008,\"y\":108.7301,\"steering_angle\":0,\"throttle\":0,\"speed\":20}]";
  string s = hasData(sdata);
  auto j = json::parse(s);
  vector<double> ptsx = j[1]["ptsx"];
  vector<double> ptsy = j[1]["ptsy"];
  double px = j[1]["x"];
  double py = j[1]["y"];
  double psi = j[1]["psi"];
  double v = j[1]["speed"];

  v *= 0.44704;

  /*
  * TODO: Calculate steering angle and throttle using MPC.
  *
  * Both are in between [-1, 1].
  *
  */

  // int N = ptsx.size();

  Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd>(ptsx.data(), ptsx.size());
  Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd>(ptsy.data(), ptsy.size());
  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

  Eigen::VectorXd state(4);
  state << px, py, psi, v;

  MPC mpc;

  vector<double> sol = mpc.Solve(state, coeffs);

  size_t nn = 25;
  int x_start     = 0;
  int y_start     = x_start + nn;
  int psi_start   = y_start + nn;
  int v_start     = psi_start + nn;
  int cte_start   = v_start + nn;
  int epsi_start  = cte_start + nn;
  int delta_start = epsi_start + nn;
  int a_start     = delta_start + nn - 1;

  double steer_value = -sol[delta_start];
  double throttle_value = sol[a_start];
  cout<<"steer "<<steer_value<<", throttle "<<throttle_value<<endl;

  cout<<"velocity: ";
  for(size_t t=0; t<nn; t++) cout<<sol[v_start+t]<<", "; cout<<endl;


  cout<<"x: ";
  for(size_t t=0; t<nn; t++) cout<<sol[x_start+t]<<", "; cout<<endl;

  cout<<"y: ";
  for(size_t t=0; t<nn; t++) cout<<sol[y_start+t]<<", "; cout<<endl;

  cout<<"cte: ";
  for(size_t t=0; t<nn; t++) cout<<sol[cte_start+t]<<", "; cout<<endl;

  cout<<"psi: ";
  for(size_t t=0; t<nn; t++) cout<<sol[psi_start+t]<<", "; cout<<endl;

  cout<<"epsi: ";
  for(size_t t=0; t<nn; t++) cout<<sol[epsi_start+t]<<", "; cout<<endl;

  cout<<"steer: ";
  for(size_t t=0; t<nn-1; t++) cout<<sol[delta_start+t]<<", "; cout<<endl;

  cout<<"throttle: ";
  for(size_t t=0; t<nn-1; t++) cout<<sol[a_start+t]<<", "; cout<<endl;
}

int main() {
  // test();
  // return 0;
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
          v *= 0.44704;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          int n_pts = ptsx.size();

          Eigen::VectorXd xvals(n_pts), yvals(n_pts);

          for(int i=0; i<n_pts; i++){
            double x = ptsx[i] - px, y = ptsy[i] - py;
            xvals[i] = x*cos(-psi) - y*sin(-psi);
            yvals[i] = x*sin(-psi) + y*cos(-psi);
          }

          Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

          Eigen::VectorXd state(4);
          state << 0, 0, 0, v;

          vector<double> sol = mpc.Solve(state, coeffs);

          size_t N = 40;
          int x_start     = 0;
          int y_start     = x_start + N;
          int psi_start   = y_start + N;
          int v_start     = psi_start + N;
          int cte_start   = v_start + N;
          int epsi_start  = cte_start + N;
          int delta_start = epsi_start + N;
          int a_start     = delta_start + N - 1;

          double steer_value = -sol[delta_start];
          double throttle_value = sol[a_start];

          cout<<"steer "<<steer_value<<", throttle "<<throttle_value<<endl;

          cout<<"velocity: ";
          for(size_t t=0; t<N; t++) cout<<sol[v_start+t]<<", "; cout<<endl;

          cout<<"cte: ";
          for(size_t t=0; t<N; t++) cout<<sol[cte_start+t]<<", "; cout<<endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals(N);
          vector<double> mpc_y_vals(N);
          for(size_t t=0; t < N; t++) {
            mpc_x_vals[t] = sol[x_start+t];
            mpc_y_vals[t] = sol[y_start+t];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(n_pts);
          vector<double> next_y_vals(n_pts);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for(int t=0; t < n_pts; t++) {
            next_x_vals[t] = xvals[t];
            next_y_vals[t] = yvals[t];
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          // this_thread::sleep_for(chrono::milliseconds(100));
          this_thread::sleep_for(chrono::milliseconds(0));
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
