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

#include "path_planner.h"
#include "vehicle.h"

#include "spline.h"

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

  // Constant value
  int num_next_vals = 50;

  // Other Vehicles
  map<int, Vehicle> vehicles;

  // Path Planner which control all of the car's behavior
  PathPlanner planner(map_waypoints_x, map_waypoints_y, map_waypoints_s);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &max_s,
               &num_next_vals, &vehicles, &planner]
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

          // Convert sensor fusion data into vehicles array
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

          /* === Planning === */
          // Step 1. Inform latest car info to planner
          planner.UpdateCarInfo(car_x, car_y, car_s, car_d, car_yaw,
            car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d);

          // Step 2. Generate candidate paths
          // Plan from previous planned path end point
          if (planner.path_queue.size() <= planner.NUM_QUEUE_PATH) {
            double search_target_x = car_x;
            double search_target_y = car_y;
            if (previous_path_x.size() > 0) {
              vector<double> xy = getXY(planner.path_queue[planner.path_queue.size() - 1][0],
                planner.path_queue[planner.path_queue.size() - 1][1],
                map_waypoints_s, map_waypoints_x, map_waypoints_y);
              search_target_x = xy[0];
              search_target_y = xy[1];
            }
            int next_waypoint_id = NextWaypoint(search_target_x, search_target_y,
                  car_yaw, map_waypoints_x, map_waypoints_y);
            // TODO(zuqqhi2): need to care road end
            if (end_path_s >= map_waypoints_s[next_waypoint_id]) {
                  next_waypoint_id = (next_waypoint_id + 1) % map_waypoints_x.size();
            }

            vector<vector<vector<double>>> candidates =
              planner.GenerateCandidatePaths(next_waypoint_id);

            // TODO(zuqqhi2): Step 3. Choose appropriate path for current situation
            vector<vector<double>> planned_path = planner.ChooseAppropriatePath(
              candidates, vehicles);
          }

          // TODO(zuqqhi2): Step 4. Attach generated next path to next_x_vals, next_y_vals
          vector<double> tmp_next_x_vals;
          vector<double> tmp_next_y_vals;

          // Register remaining path
          vector<vector<double>> path =
            planner.GetPlannedPath(num_next_vals - previous_path_x.size());
          for (int i = 0; i < previous_path_x.size(); ++i) {
            tmp_next_x_vals.push_back(previous_path_x[i]);
            tmp_next_y_vals.push_back(previous_path_y[i]);
          }

          // Add new planned path
          const int num_additional_points = 5;
          bool is_there_new_path_entry = false;
          int new_path_first_idx = -1;
          for (int i = 0; i < path.size(); i++) {
            if (path[i].size() > 2) {
              is_there_new_path_entry = true;
              new_path_first_idx = previous_path_x.size() + i;
            }

            vector<double> xy = getXY(path[i][0],
              path[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // Interpolation to make path smooth
            if (tmp_next_x_vals.size() > 0) {
              double v = sqrt(std::pow(xy[0] - tmp_next_x_vals[tmp_next_x_vals.size() - 1], 2.0)
                + std::pow(xy[1] - tmp_next_y_vals[tmp_next_x_vals.size() - 1], 2.0));
              // std::cout << i << ": " << v << ", " << car_speed
              //  << ", " << xy[0] << ", " << car_x << std::endl;
              if (v > 22.0) {  // 22.352 m/s (50MPH)
                std::cout << " here" << std::endl;
                tmp_next_x_vals.push_back((xy[0]
                  + tmp_next_x_vals[tmp_next_x_vals.size() - 1]) / 2.0);
                tmp_next_y_vals.push_back((xy[1]
                  + tmp_next_y_vals[tmp_next_y_vals.size() - 1]) / 2.0);
              }
            }

            tmp_next_x_vals.push_back(xy[0]);
            tmp_next_y_vals.push_back(xy[1]);
          }
          // std::cout << std::endl;

          // TODO(zuqqhi2): Implement interpolation for missing points
          if (previous_path_x.size() > 0 && is_there_new_path_entry) {
            vector<vector<double>> path_additional =
              planner.GetPlannedPath(num_additional_points);
            for (int i = 0; i < path_additional.size(); i++) {
              vector<double> xy = getXY(path_additional[i][0],
                path_additional[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
              tmp_next_x_vals.push_back(xy[0]);
              tmp_next_y_vals.push_back(xy[1]);
            }

            const int num_before_new_points = 2;
            const int num_end_points = 2;
            vector<double> tmp2_next_x_vals, tmp2_next_y_vals;
            for (int i = new_path_first_idx - num_before_new_points; i < new_path_first_idx; i++) {
              tmp2_next_x_vals.push_back(tmp_next_x_vals[i]);
              tmp2_next_y_vals.push_back(tmp_next_y_vals[i]);
            }
            for (int i = tmp_next_x_vals.size() - num_end_points;
              i < tmp_next_x_vals.size(); i++) {
              tmp2_next_x_vals.push_back(tmp_next_x_vals[i]);
              tmp2_next_y_vals.push_back(tmp_next_y_vals[i]);
            }

            const int n = num_before_new_points + num_end_points;
            vector<double> p(n), tmp3_next_x_vals(n), tmp3_next_y_vals(n);
            std::iota(p.begin(), p.end(), 0);
            std::sort(p.begin(), p.end(), [&](int a, int b) {
              return tmp2_next_x_vals[a] < tmp2_next_x_vals[b];
            });
            for (int i = 0; i < tmp2_next_x_vals.size(); i++) {
              tmp3_next_x_vals[i] = tmp2_next_x_vals[p[i]];
              tmp3_next_y_vals[i] = tmp2_next_y_vals[p[i]];
            }

            tk::spline sp;
            sp.set_points(tmp3_next_x_vals, tmp3_next_y_vals);

            for (int i = 0; i < tmp_next_x_vals.size(); i++) {
              next_x_vals.push_back(tmp_next_x_vals[i]);
              if (i >= new_path_first_idx - num_before_new_points) {
                next_y_vals.push_back(sp(tmp_next_x_vals[i]));
              } else {
                next_y_vals.push_back(tmp_next_y_vals[i]);
              }
            }
          } else {
            for (int i = 0; i < tmp_next_x_vals.size(); i++) {
              next_x_vals.push_back(tmp_next_x_vals[i]);
              next_y_vals.push_back(tmp_next_y_vals[i]);
            }
          }

          // To get path switch point smooth
          // do spline interpolation with first n points and end n points
          // TODO(zuqqhi2): Interpolation for more stable(everytime update previous path now)
          /*
          const int num_points = 2;
          const int n = num_points * 2;

          vector<double> tmp2_next_x_vals, tmp2_next_y_vals;
          for (int i = 0; i < num_points; i++) {
            tmp2_next_x_vals.push_back(tmp_next_x_vals[i]);
            tmp2_next_y_vals.push_back(tmp_next_y_vals[i]);
          }
          for (int i = tmp_next_x_vals.size() - 1 - num_points; i < tmp_next_x_vals.size(); i++) {
            tmp2_next_x_vals.push_back(tmp_next_x_vals[i]);
            tmp2_next_y_vals.push_back(tmp_next_y_vals[i]);
          }

          vector<double> p(n), tmp3_next_x_vals(n), tmp3_next_y_vals(n);
          std::iota(p.begin(), p.end(), 0);
          std::sort(p.begin(), p.end(), [&](int a, int b) {
            return tmp2_next_x_vals[a] < tmp2_next_x_vals[b];
          });
          for (int i = 0; i < tmp2_next_x_vals.size(); i++) {
            tmp3_next_x_vals[i] = tmp2_next_x_vals[p[i]];
            tmp3_next_y_vals[i] = tmp2_next_y_vals[p[i]];
          }

          tk::spline sp;
          sp.set_points(tmp3_next_x_vals, tmp3_next_y_vals);

          for (int i = 0; i < tmp_next_x_vals.size(); i++) {
            next_x_vals.push_back(tmp_next_x_vals[i]);
            next_y_vals.push_back(sp(tmp_next_x_vals[i]));
          }
          */
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
