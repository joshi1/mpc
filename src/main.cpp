#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "mpc.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double radius1;
double radius2;
bool debugging_enabled = false;

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

void printXY(string str, vector<double> x_vals, vector<double> y_vals){
  if(debugging_enabled == false){
    return;
  }
  cout << str << endl;
  for (int i = 0; i < x_vals.size(); i++){
    cout << "(" << x_vals[i] << "," << y_vals[i] << ")"<< endl;
  }
}

double radiusOfCurvature(Eigen::VectorXd coeffs, double x){

  double ddxy = 3*coeffs[3]*pow(x, 2) + 2*coeffs[2]*x + coeffs[1];
  double ddxy2 = pow(ddxy, 2);
  double ddx2y2 = 6*coeffs[3]*x + 2*coeffs[2];
  double radius = pow(1+ddxy2, 1.5)/ddx2y2;
  return radius;
  
}
int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  int iter = 0;
  h.onMessage([&mpc, &iter](uWS::WebSocket<uWS::SERVER> ws, char *data,
			   size_t length,
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
	  DEBUG("*************** Iter " << iter);
	  iter++;
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];
	  double s   = j[1]["steering_angle"];
	  double a   = j[1]["throttle"];

	  
	  //To take care of the latency in the system of 100msecs.
	  //The idea is to predict where the car will be in 100msecs and use
	  //it as the current location of the car, psi and v.
	  //  Note that there is another way to do this where , in mpc.cpp,
	  //  we could return the predicted delta from the solution for the
	  //  future state equal to the latency in the system. However, in this
	  //  method there is a dependency on the chosen dt - if the system latency
	  //  is not an integer multiple of dt, then the method does not
	  //  really work too well. 
	  double l = 0.125; 
	  px  = px + v*cos(psi)*l;
	  py  = py + v*sin(psi)*l;
	  psi = psi - v*(s/Lf)*l;
	  v   = v + a*l;
	  
	  // Tranform from map to vehicle coordinates
	  Eigen::VectorXd ptsx_v(ptsx.size());
	  Eigen::VectorXd ptsy_v(ptsy.size());
	  for(int i = 0; i < ptsx.size(); ++i) {
	    double x_shift = ptsx[i] - px;
	    double y_shift = ptsy[i] - py;
	    ptsx_v[i] = (x_shift * cos(-psi) - y_shift * sin(-psi));
	    ptsy_v[i] = (x_shift * sin(-psi) + y_shift * cos(-psi));
	  }

	  //Fit to a polynomial in the vehicle coordinate system
	  auto coeffs = polyfit(ptsx_v, ptsy_v, 3);
	  
	  //double cte = polyeval(coeffs, px) - py;
	  double cte = polyeval(coeffs, 0) - 0;

	  //epsi
	  double epsi = 0 - atan(coeffs[1]);
	  
	  //Take radius of curvature into account to minimize
	  // the acceleration acutator in the cost function.
	  //  (detailed explanation in MPC.cpp)
	  //This is best done in the maps coordinate system for 
	  // the waypoints.
	  Eigen::VectorXd new_x(ptsx.size());
	  Eigen::VectorXd new_y(ptsy.size());
	  for(int i = 0; i < ptsx.size(); ++i) {
	    new_x[i] = ptsx[i];
	    new_y[i] = ptsy[i];
	  }
	  auto coeffs2 = polyfit(new_x, new_y, 3);
	  //Tried to experiment to look ahead a little (new_x[3].
	  // However, radius1 seems to perform slightly better.
	  radius1 = radiusOfCurvature(coeffs2, new_x[0]);
	  radius2 = radiusOfCurvature(coeffs2, new_x[3]);

	  
	  //Create state vector to feed into solve
	  Eigen::VectorXd state(6);
	  //state << px, py, psi, v, cte, epsi;
	  state << 0, 0, 0, v, cte, epsi;
	  
	  auto vars = mpc.Solve(state, coeffs);

	  //Calculate steering angle and throttle using MPC.
	  // steering value between [-1, 1]
          double steer_value = vars[4]/deg2rad(25);
          double throttle_value = vars[5];

	  printf(" steer value = % 06.5f, throttle value = %06.5f \n",
		 steer_value, throttle_value);
	  
          json msgJson;
	  
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

	  int N = vars[6];
	  //DEBUG("*** N:" << N);

	  int base_xy = 7;
	  for (int i=1; i <= N; i += 1){
	    mpc_x_vals.push_back(vars[base_xy]);
	    mpc_y_vals.push_back(vars[base_xy + 1]);
	    base_xy += 2;
	  }
	  
          //.. add (x,y) points to list here, points are in reference
	  // to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

	  for (int i = 0; i < ptsx_v.size(); i++) {
	    next_x_vals.push_back(ptsx_v[i]);
	    next_y_vals.push_back(ptsy_v[i]);
	  }

          //.. add (x,y) points to list here, points are in reference
	  // to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

	  printXY("Way-points(MAP coordinate system)",
		  ptsx, ptsy);

	  DEBUG("   px:" << px);
	  DEBUG("   py:" << py);
	  DEBUG("  psi:" << psi);
	  DEBUG("    v:" << v);
	  DEBUG("   r1:" << radius1);
	  DEBUG("   r2:" << radius2);
	  
	  printXY("Way-points(Vehicle coordinate system)",
		  next_x_vals, next_y_vals);

	  printXY("MPC points",
		  mpc_x_vals, mpc_y_vals);

	  DEBUG("    steer value:" << steer_value);
	  DEBUG(" throttle value:" << throttle_value);

	  //return -1;
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
