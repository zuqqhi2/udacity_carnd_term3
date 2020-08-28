#include <cmath>
#include <iostream>  // will be removed
#include <iomanip>  // will be removed

#include <algorithm>
#include "path_planner.h"

// Setup cost function set with weight, min/max value for 0-1 scaling
PathPlanner::PathPlanner(const vector<double> &map_waypoints_x,
     const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s)
     : map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
     map_waypoints_s(map_waypoints_s), cur_velocity(0.0), end_path_state(STATE_NORMAL) {
     cost_functions[0] = new CollisionCostFunction(COST_WEIGHT_COLLISION, UNIT_TIME, MS_2_MPH,
          MAX_FUTURE_REFERENCE_S, static_cast<int>(LANE_WIDTH), VEHICLE_RADIUS);
     cost_functions[1] = new VehicleBufferCostFunction(COST_WEIGHT_VEHICLE_BUFFER, UNIT_TIME,
          MS_2_MPH, MAX_FUTURE_REFERENCE_S, static_cast<int>(LANE_WIDTH), VEHICLE_RADIUS);
     cost_functions[2] = new SlowCostFunction(COST_WEIGHT_D_STATE_DIFF, UNIT_TIME,
          MS_2_MPH, MAX_FUTURE_REFERENCE_S, static_cast<int>(LANE_WIDTH), MAX_VELOCITY);
     cost_functions[3] = new DiffDStateCostFunction(COST_WEIGHT_D_STATE_DIFF, UNIT_TIME,
          MS_2_MPH, MAX_FUTURE_REFERENCE_S, static_cast<int>(LANE_WIDTH));
}

// Update state
void PathPlanner::UpdateState() {
     if (this->end_path_state != this->STATE_NORMAL
          && this->end_path_state != this->STATE_NORMAL_SLOW) {
          if (abs(this->GetLaneCenter(this->future_target_lane) - this->car_d) < 1e-3) {
               this->end_path_state = this->STATE_NORMAL;
               return;
          }
     }

     // Speed down before lane change
     if (this->end_path_state == this->STATE_PREPARE_LANE_CHANGE
          && this->cur_velocity < this->MAX_VELOCITY * this->MAX_LANE_CHANGE_VELOCITY_DOWN_RATE
          && this->cur_velocity > this->MAX_VELOCITY * this->MIN_LANE_CHANGE_VELOCITY_DOWN_RATE) {
          this->end_path_state = this->STATE_LANE_CHANGE;
          return;
     }
}

// Get latest my car info
void PathPlanner::UpdateCarInfo(double x, double y, double s, double d, double yaw,
     double speed, const vector<double> &prev_path_x, const vector<double> &prev_path_y,
     double end_path_s, double end_path_d, const map<int, Vehicle> &vehicles) {
     this->car_x = x;
     this->car_y = y;
     this->car_s = s;
     this->car_d = d;
     this->car_yaw = yaw;
     this->car_speed = speed;
     this->previous_path_x = prev_path_x;
     this->previous_path_y = prev_path_y;
     this->vehicles = vehicles;

     if (this->previous_path_x.size() == 0) {
          this->end_path_s = this->car_s;
          this->end_path_d = this->car_d;
     } else {
          this->end_path_s = end_path_s;
          this->end_path_d = end_path_d;
     }

     // Check end point lane of last predicted path
     this->end_path_lane = this->GetLaneId(this->end_path_d);

     // Update state
     this->UpdateState();
}

// Update speed
void PathPlanner::UpdateSpeed() {
     // Keep max speed
     if (this->end_path_state == this->STATE_NORMAL) {
          if (this->cur_velocity < this->MAX_VELOCITY) {
               this->cur_velocity += this->VELOCITY_STEP;
          }
     // Slow down and keep a slow speed for lane change and normal slow
     } else if (this->end_path_state == this->STATE_PREPARE_LANE_CHANGE
          || this->end_path_state == this->STATE_NORMAL_SLOW) {
          if (this->cur_velocity > this->MAX_VELOCITY * this->MAX_LANE_CHANGE_VELOCITY_DOWN_RATE) {
               this->cur_velocity -= this->VELOCITY_STEP;
          }
          if (this->cur_velocity < this->MAX_VELOCITY * this->MIN_LANE_CHANGE_VELOCITY_DOWN_RATE) {
               this->cur_velocity += this->VELOCITY_STEP;
          }
     }
}

// Generate previous path to make smooth future path
vector<vector<double>> PathPlanner::GeneratePreviousPath(double (*deg2rad)(double)) {
     // Provided previous path point size.
     int prev_size = this->previous_path_x.size();

     // Preventing collitions.
     if (prev_size > 0) { this->car_s = this->end_path_s; }

     vector<vector<double>> res;

     double ref_x = this->car_x;
     double ref_y = this->car_y;
     double ref_yaw = deg2rad(this->car_yaw);

     // Do I have have previous points
     if (prev_size < 2) {
          // Use estimated previous point
          double prev_car_x = this->car_x - cos(ref_yaw);
          double prev_car_y = this->car_y - sin(ref_yaw);

          vector<double> pts1 = {prev_car_x, prev_car_y};
          vector<double> pts2 = {this->car_x, this->car_y};
          res.push_back(pts1);
          res.push_back(pts2);
     } else {
          // Use the last two points
          ref_x = previous_path_x[prev_size - 1];
          ref_y = previous_path_y[prev_size - 1];

          double ref_x_prev = previous_path_x[prev_size - 2];
          double ref_y_prev = previous_path_y[prev_size - 2];
          ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          vector<double> pts1 = {ref_x_prev, ref_y_prev};
          vector<double> pts2 = {ref_x, ref_y};
          res.push_back(pts1);
          res.push_back(pts2);
     }

     return res;
}

// Pick up lowest cost plan and generate rough path
vector<vector<double>> PathPlanner::GenerateFuturePoints(const double ref_x, const double ref_y,
     const double ref_yaw, const vector<vector<double>> &pts, vector<double> (*getXY)(double,
     double, const vector<double>&, const vector<double>&, const vector<double>&)) {

     vector<vector<double>> best_path;
     // Register previous path
     for (int i = 0; i < pts.size(); i++) {
          best_path.push_back(pts[i]);
     }

     if (this->end_path_state == this->STATE_NORMAL
          || this->end_path_state == this->STATE_NORMAL_SLOW) {
          // Generate candidate path points to choose best path
          vector<vector<vector<double>>> candidate_paths;
          vector<double> velocities;
          vector<int> states;
          for (int dl = -1; dl <= 1; dl++) {
               int target_lane = this->end_path_lane + dl;
               if (target_lane < 0 || target_lane >= this->NUM_LANES) { continue; }

               vector<vector<double>> path;
               path.push_back({this->car_s, this->end_path_d});
               for (int i = 1; i <= this->NUM_FUTURE_REFERENCE_PATH_POINTS; i++) {
                    vector<double> pts_sd = {this->car_s + i * this->MAX_FUTURE_REFERENCE_S,
                         this->GetLaneCenter(target_lane)};
                    path.push_back(pts_sd);
               }
               candidate_paths.push_back(path);
               if (dl == 0) {
                    velocities.push_back((this->MAX_VELOCITY + this->cur_velocity) / 2.0);
                    states.push_back(this->STATE_NORMAL);
               } else {
                    velocities.push_back((this->MAX_VELOCITY
                         * this->MAX_LANE_CHANGE_VELOCITY_DOWN_RATE + this->cur_velocity) / 2.0);
                    states.push_back(this->STATE_PREPARE_LANE_CHANGE);
               }
          }
          // Slow down path
          vector<vector<double>> tmp_path;
          tmp_path.push_back({this->car_s, this->end_path_d});
          for (int i = 1; i <= this->NUM_FUTURE_REFERENCE_PATH_POINTS; i++) {
               vector<double> pts_sd = {this->car_s + i * this->MAX_FUTURE_REFERENCE_S,
                    this->GetLaneCenter(this->end_path_lane)};
               tmp_path.push_back(pts_sd);
          }
          candidate_paths.push_back(tmp_path);
          velocities.push_back((this->MAX_VELOCITY
               * this->MAX_LANE_CHANGE_VELOCITY_DOWN_RATE + this->cur_velocity) / 2.0);
          states.push_back(this->STATE_NORMAL_SLOW);

          // Find lowest cost path
          double min_cost = 1e+6;
          double min_cost_idx = 0;
          for (int i = 0; i < candidate_paths.size(); i++) {
               vector<vector<double>> path = candidate_paths[i];

               // Calculate cost of given path
               double total_cost = 0.0;
               for (int j = 0; j < this->NUM_COST_FUNCTIONS; j++) {
                    total_cost += this->cost_functions[j]->CalculateCost(path,
                         this->vehicles, this->previous_path_x.size(), velocities[i]);
               }

               if (total_cost < min_cost) {
                    min_cost = total_cost;
                    min_cost_idx = i;
               }

               // Debug
               std::cout << "  > " << min_cost
                    << ", " << total_cost
                    << ", " << path[path.size() - 1][1]
                    << std::endl;
          }

          // Update state
          vector<vector<double>> min_cost_path = candidate_paths[min_cost_idx];
          int new_target_lane = this->GetLaneId(min_cost_path[min_cost_path.size() - 1][1]);
          this->end_path_state = states[min_cost_idx];
          if (this->end_path_state == this->STATE_PREPARE_LANE_CHANGE) {
               this->future_target_lane = new_target_lane;
          }

          // Generate best path(xy)
          // Register future points to the reference point list
          for (int i = 1; i <= this->NUM_FUTURE_REFERENCE_PATH_POINTS; i++) {
               vector<double> pts_xy = getXY(min_cost_path[i][0], min_cost_path[i][1],
                    this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
               best_path.push_back(pts_xy);
          }
     // If not normal state, just follow already decided plan
     } else {
          int target_lane = this->future_target_lane;
          if (this->end_path_state == this->STATE_PREPARE_LANE_CHANGE) {
               target_lane = this->end_path_lane;
          }
          for (int i = 1; i <= this->NUM_FUTURE_REFERENCE_PATH_POINTS; i++) {
               vector<double> pts_sd = {this->car_s + i * this->MAX_FUTURE_REFERENCE_S,
                    this->GetLaneCenter(target_lane)};
               vector<double> pts_xy = getXY(pts_sd[0], pts_sd[1],
                    this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
               best_path.push_back(pts_xy);
          }
     }

     // Making coordinates to the local car coordinates
     for ( int i = 0; i < best_path.size(); i++ ) {
          double shift_x = best_path[i][0] - ref_x;
          double shift_y = best_path[i][1] - ref_y;

          best_path[i][0] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          best_path[i][1] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
     }

     return best_path;
}

// Generate complete path from previous path and rough future path
vector<vector<double>> PathPlanner::GenerateSmoothPath(
     const vector<vector<double>> &ref_pts, double ref_x, double ref_y, double ref_yaw) {
     // Create the spline.
     vector<double> ptsx;
     vector<double> ptsy;

     for (int i = 0; i < ref_pts.size(); i++) {
          ptsx.push_back(ref_pts[i][0]);
          ptsy.push_back(ref_pts[i][1]);
     }

     tk::spline s;
     s.set_points(ptsx, ptsy);

     // Generate smooth path
     double target_x = this->MAX_FUTURE_REFERENCE_S;
     double target_y = s(target_x);
     double target_dist = sqrt(target_x * target_x + target_y * target_y);

     double x_add_on = 0;

     vector<vector<double>> path;
     for (int i = 1; i < this->NUM_PATH_POINTS - this->previous_path_x.size(); i++) {
          double N = target_dist / (this->UNIT_TIME * this->cur_velocity / this->MS_2_MPH);
          double x_point = x_add_on + target_x / N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          x_point += ref_x;
          y_point += ref_y;

          path.push_back({x_point, y_point});
     }

     return path;
}

// Generate best future path
vector<vector<double>> PathPlanner::GenerateBestPath(
     double (*deg2rad)(double), vector<double> (*getXY)(double, double,
     const vector<double>&, const vector<double>&, const vector<double>&)) {

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

     // Generate best future path
     vector<vector<double>> prev_path = this->GeneratePreviousPath(deg2rad);
     vector<vector<double>> future_pts = this->GenerateFuturePoints(
          ref_x, ref_y, ref_yaw, prev_path, getXY);

     // Complete future path with spline curve
     return this->GenerateSmoothPath(future_pts, ref_x, ref_y, ref_yaw);
}
