#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

#include "path_planner.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Other Vehicles
  map<int, Vehicle> vehicles;

  // Path Planner which control all of the car's behavior
  PathPlanner planner(map_waypoints_x, map_waypoints_y, map_waypoints_s);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &max_s,
               &vehicles, &planner]  // These 2 objects are original classes
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        // std::cout << s << std::endl;  // For check sensor data
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /* === Start Planning === */

          // Step 1. Convert sensor fusion data into vehicles array
          for (int i = 0; i < sensor_fusion.size(); i++) {
            int v_id = sensor_fusion[i][0];
            double x[2] = {sensor_fusion[i][1], sensor_fusion[i][3]};
            double y[2] = {sensor_fusion[i][2], sensor_fusion[i][4]};
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];

            // Ignore vehicles on the other lane
            if (d < 0.0) { continue; }

            // Register in case of new vehicle, otherwise update
            if (vehicles.find(v_id) != vehicles.end()) {
              vehicles[v_id].UpdateState(x, y, s, d);
            } else {
              vehicles[v_id] = Vehicle(v_id, x, y, s, d);
            }
          }

          // Step 2. Inform latest car info to planner
          planner.UpdateCarInfo(car_x, car_y, car_s, car_d, car_yaw,
            car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d);

          planner.UpdateSpeed();

          // For proceeding parts
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          if (previous_path_x.size() >= 2) {
            ref_x = previous_path_x[previous_path_x.size() - 1];
            ref_y = previous_path_y[previous_path_x.size() - 1];

            double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
            double ref_y_prev = previous_path_y[previous_path_x.size() - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Step 3. Generate best future path and spline fitting
          vector<vector<double>> prev_path = planner.GeneratePreviousPath();
          vector<vector<double>> future_pts = planner.GenerateBestPath(
            ref_x, ref_y, ref_yaw, prev_path, getXY);

          // Step 4. Complete future path with spline curve
          // Calculate distance y position on 30 m ahead.
          // Provided previous path point size.
          for ( int i = 0; i < previous_path_x.size(); i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          vector<vector<double>> future_path = planner.GenerateSmoothPath(
            future_pts, ref_x, ref_y, ref_yaw);
          for (int i = 0; i < future_path.size(); i++) {
            next_x_vals.push_back(future_path[i][0]);
            next_y_vals.push_back(future_path[i][1]);
          }

          // Debug
          std::cout << std::fixed;
          std::cout << std::setprecision(2);
          std::cout << "prev = (" << car_x << ", " << car_y
            << "), next = (" << next_x_vals[0] << ", " << next_y_vals[0]
            << "), diff = " << std::sqrt(std::pow(next_x_vals[0] - car_x, 2.0)
              + std::pow(next_y_vals[0] - car_y, 2.0))
            << std::endl;

          /* === End Planning === */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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
