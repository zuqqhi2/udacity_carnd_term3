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

  // Previous vechiles state for vs, vd, as, ad calculation
  double prev_car_s = 0.0;
  double prev_car_d = 0.0;
  double prev_car_vs = 0.0;
  double prev_car_vd = 0.0;

  // Vehicles
  map<int, Vehicle> vehicles;
  
  PathPlanner planner;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &max_s,
               &prev_car_s, &prev_car_d, &prev_car_vs, &prev_car_vd, &vehicles, &planner]
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

            if (vehicles.find(v_id) != vehicles.end()) {
              vehicles[v_id].UpdateState(x, y, s, d);
            } else {
              double s_arr[3] = {s, 0.0, 0.0};
              double d_arr[3] = {d, 0.0, 0.0};
              vehicles[v_id] = Vehicle(v_id, x, y, s_arr, d_arr);
            }
          }

          // Find minimum cost target vehicle
          int num_div = 100;

          double car_vs = car_s - prev_car_s;
          double car_as = car_vs - prev_car_vs;
          vector<double> start_s = {car_s, car_vs, car_as};
          double car_vd = car_d - prev_car_d;
          double car_ad = car_vd - prev_car_vd;
          vector<double> start_d = {car_d, car_vd, car_ad};

          // Find nearest and no cost vehicle
          int min_id = 0;
          double goal_time = 1e+6;
          double global_min_cost = 1e+6;
          vector<double> global_min_cost_coef_s;
          vector<double> global_min_cost_coef_d;
          for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
            Vehicle v = item->second;

            vector<double> end_s = {v.s_state[0], v.s_state[1], v.s_state[2]};
            vector<double> end_d = {v.d_state[0], v.d_state[1], v.d_state[2]};
            // Loop between 2.0 ~ 10.0 with 1.0 step
            for (int i = 0; i < 9; i++) {
              double t = 2.0 + 1.0 * i;
              vector<double> coef_s = planner.CalculateJerkMinimizingCoef(start_s, end_s, t);
              vector<double> coef_d = planner.CalculateJerkMinimizingCoef(start_d, end_d, t);
              vector<double> target_vehicle_state = 
                {end_s[0], end_s[1], end_s[2], end_d[0], end_d[1], end_d[2]};
              double cost = planner.CalculateCost(coef_s,
                coef_d, target_vehicle_state, vehicles, num_div, t, max_s);
              if (cost < global_min_cost) {
                global_min_cost = cost;
                goal_time = t;
                min_id = v.id;
                global_min_cost_coef_s = coef_s;
                global_min_cost_coef_d = coef_d;
              }
            }
          }
          Vehicle target_vehicle = vehicles[min_id];
          std::cout << "(" << car_s << ", " << car_d << "), (" << global_min_cost << ", " << min_id << ")" << std::endl;

          // To move smoothly, keep using previous generated path
          /*
          int path_size = previous_path_x.size();
          for (int i = std::max(0, path_size - 5); i < path_size; i++) {
          // for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          */

          // Add new planned path
          // waypoints: 50
          //   => 20 ms * 50 => generates until 1s future
          //   only 50 - path_size waypoints are generated to keep 50 future waypoints
          // for (int i = 50 - path_size; i <= 50; i++) {
          for (int i = 0; i <= 50; i++) {
            double t = i / 1000.0 * 20.0;  // 20 ms
            double new_s = planner.CalculateEqRes(global_min_cost_coef_s, t);
            double new_d = planner.CalculateEqRes(global_min_cost_coef_d, t);

            vector<double> xy = getXY(new_s,
              new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }

          prev_car_s = car_s;
          prev_car_d = car_d;
          prev_car_vs = car_vs;
          prev_car_vd = car_vd;

          /*
          double dist_inc = 0.1;
          double one_step_diff_x = (target_vehicle.x - car_x) / 100.0;
          double one_step_diff_y = (target_vehicle.y - car_y) / 100.0;
          for (int i = 0; i < 100; ++i) {
            next_x_vals.push_back(car_x + one_step_diff_x * i);
            next_y_vals.push_back(car_y + one_step_diff_y * i);
          }
          */

          // === start Simple Move Forward which is explained at Getting Started lecture ===
          /*
          double dist_inc = 0.5;
          vector<double> sd = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          for (int i = 0; i < 100; ++i) {
            // Looks half of the car width is d=0.5
            vector<double> xy = getXY(sd[0] + (dist_inc * i), 2.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);

            //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          */
          // === end ===

          // === start 3.More COmplex Paths ===
          /*
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          }

          double dist_inc = 0.5;
          for (int i = 0; i < 50-path_size; ++i) {    
            next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
            next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
            pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
          }
          */
          // === end ===

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
