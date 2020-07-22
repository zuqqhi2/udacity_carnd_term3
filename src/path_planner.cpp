#include <cmath>
#include <iostream>  // will be removed
#include <iomanip>  // will be removed

#include <algorithm>
#include "path_planner.h"

// Setup cost function set with weight, min/max value for 0-1 scaling
PathPlanner::PathPlanner(const vector<double> &map_waypoints_x,
     const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s)
     : map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
     map_waypoints_s(map_waypoints_s), cur_velocity(0.0) {
     cost_functions[0] = new CollisionCostFunction(COST_WEIGHT_COLLISION, VEHICLE_RADIUS);
     cost_functions[1] = new VehicleBufferCostFunction(COST_WEIGHT_VEHICLE_BUFFER, VEHICLE_RADIUS);
     cost_functions[2] = new DiffDStateCostFunction(COST_WEIGHT_D_STATE_DIFF);
     cost_functions[3] = new GoalArriveTimeCostFunction(COST_WEIGHT_GOAL_ARRIVE_TIME, MAX_SPEED);
     cost_functions[4] = new DiffSpeedCostFunction(COST_WEIGHT_DIFF_SPEED);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::GetXYFromSD(double s, double d,
     const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
     int prev_wp = -1;

     while (s > maps_s[prev_wp + 1] && (prev_wp < static_cast<int>(maps_s.size() - 1))) {
          ++prev_wp;
     }

     int wp2 = (prev_wp + 1) % maps_x.size();

     double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
     // the x, y, s along the segment
     double seg_s = (s - maps_s[prev_wp]);

     double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
     double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

     double perp_heading = heading - M_PI / 2;

     double x = seg_x + d * cos(perp_heading);
     double y = seg_y + d * sin(perp_heading);

     return {x, y};
}


double PathPlanner::CalculatePolynomialResult(const vector<double> &x, double t) {
     double total = 0.0;
     for (int i = 0; i < x.size(); i++) {
          total += x[i] * std::pow(t, static_cast<double>(i));
     }

     return total;
}

// Get latest my car info
void PathPlanner::UpdateCarInfo(double x, double y, double s, double d, double yaw,
     double speed, const vector<double> &prev_path_x, const vector<double> &prev_path_y,
     double end_path_s, double end_path_d) {
     this->car_x = x;
     this->car_y = y;
     this->car_s = s;
     this->car_d = d;
     this->car_yaw = yaw;
     this->car_speed = speed;
     this->previous_path_x = prev_path_x;
     this->previous_path_y = prev_path_y;
     this->end_path_s = end_path_s;
     this->end_path_d = end_path_d;
}

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

vector<vector<double>> PathPlanner::GenerateFuturePoints(const double ref_x, const double ref_y,
        const double ref_yaw, const vector<vector<double>> &pts, vector<double> (*getXY)(double,
        double, const vector<double>&, const vector<double>&, const vector<double>&)) {

     vector<vector<double>> path;

     for (int i = 0; i < pts.size(); i++) {
          path.push_back(pts[i]);
     }

     for (int i = 1; i <= 3; i++) {
          vector<double> pts_sd = {this->car_s + i * 30.0, 2 + 4 * 1};
          vector<double> pts_xy = getXY(pts_sd[0], pts_sd[1],
               this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
          path.push_back(pts_xy);
     }

     // Making coordinates to local car coordinates.
     for ( int i = 0; i < path.size(); i++ ) {
          double shift_x = path[i][0] - ref_x;
          double shift_y = path[i][1] - ref_y;

          path[i][0] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          path[i][1] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
     }

     return path;
}

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
     double target_x = 30.0;
     double target_y = s(target_x);
     double target_dist = sqrt(target_x * target_x + target_y * target_y);

     double x_add_on = 0;

     vector<vector<double>> path;
     for (int i = 1; i < 50 - this->previous_path_x.size(); i++) {
          double N = target_dist / (0.02 * this->cur_velocity / 2.24);
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

vector<vector<double>> PathPlanner::GenerateBestPath(
     double (*deg2rad)(double), vector<double> (*getXY)(double, double,
     const vector<double>&, const vector<double>&, const vector<double>&),
     const map<int, Vehicle> &vehicles) {

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
     vector<vector<double>> prev_path = this->GeneratePreviousPath(deg2rad);
     vector<vector<double>> future_pts = this->GenerateFuturePoints(
          ref_x, ref_y, ref_yaw, prev_path, getXY);

     // Step 4. Complete future path with spline curve
     return this->GenerateSmoothPath(future_pts, ref_x, ref_y, ref_yaw);
}

// Generate candidates path
// Number of candidates: 4 (left, straight, right, slow down)
// ex) L1 -> L2 (Changing lane to right)
// ============================
//  *++++++p1+++  p2     p3     (L1)
// ----------------------------
//         p1   ++p2+++++p3     (L2)
// ----------------------------
//         p1     p2     p3     (L3)
// ----------------------------
vector<vector<vector<double>>> PathPlanner::GenerateCandidatePaths() {
     double ref_vel = 49.5;  // MPH
     int lane = 1;

     // Provided previous path point size.
     int prev_size = this->previous_path_x.size();

     // Preventing collitions.
     if (prev_size > 0) { this->car_s = this->end_path_s; }

     vector<double> ptsx;
     vector<double> ptsy;

     double ref_x = this->car_x;
     double ref_y = this->car_y;
     double ref_yaw = Degree2Radian(this->car_yaw);

     // Do I have have previous points
     if ( prev_size < 2 ) {
          // There are not too many...
          double prev_car_x = car_x - cos(ref_yaw);
          double prev_car_y = car_y - sin(ref_yaw);

          ptsx.push_back(prev_car_x);
          ptsx.push_back(car_x);

          ptsy.push_back(prev_car_y);
          ptsy.push_back(car_y);
     } else {
          // Use the last two points.
          ref_x = previous_path_x[prev_size - 1];
          ref_y = previous_path_y[prev_size - 1];

          double ref_x_prev = previous_path_x[prev_size - 2];
          double ref_y_prev = previous_path_y[prev_size - 2];
          ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
     }

     // Setting up target points in the future.
     vector<double> next_wp0 = this->GetXYFromSD(car_s + 30,
          2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
     vector<double> next_wp1 = this->GetXYFromSD(car_s + 60,
          2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
     vector<double> next_wp2 = this->GetXYFromSD(car_s + 90,
          2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

     ptsx.push_back(next_wp0[0]);
     ptsx.push_back(next_wp1[0]);
     ptsx.push_back(next_wp2[0]);

     ptsy.push_back(next_wp0[1]);
     ptsy.push_back(next_wp1[1]);
     ptsy.push_back(next_wp2[1]);

     // Making coordinates to local car coordinates.
     for (int i = 0; i < ptsx.size(); i++) {
          double shift_x = ptsx[i] - ref_x;
          double shift_y = ptsy[i] - ref_y;

          ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
     }

     // Create the spline.
     tk::spline s;
     s.set_points(ptsx, ptsy);

     // Calculate distance y position on 30 m ahead.
     double target_x = 30.0;
     double target_y = s(target_x);
     double target_dist = sqrt(target_x*target_x + target_y*target_y);

     double x_add_on = 0;

     vector<vector<vector<double>>> candidates;
     vector<vector<double>> new_path_xy;
     double N = target_dist / (0.02 * ref_vel / 2.24);
     for (int i = 1; i < 50 - prev_size; i++) {
          double x_point = x_add_on + target_x / N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          x_point += ref_x;
          y_point += ref_y;

          new_path_xy.push_back({x_point, y_point});
     }
     candidates.push_back(new_path_xy);

     /*
     vector<double> new_s = {plan_start_s};
     for (int i = 0; i < NUM_WAYPOINTS_USED_FOR_PATH; i++) {
          new_s.push_back(this->map_waypoints_s[next_waypoint_ids[i]]);
     }

     double new_end_d[NUM_ACTIONS] = {
          // Move to left lane
          static_cast<int>((plan_start_d / LANE_WIDTH) - 1) * LANE_WIDTH + LANE_WIDTH / 2.0,
          // No change
          static_cast<int>(plan_start_d / LANE_WIDTH) * LANE_WIDTH + LANE_WIDTH / 2.0,
          // Move to right lane
          static_cast<int>((plan_start_d / LANE_WIDTH) + 1) * LANE_WIDTH + LANE_WIDTH / 2.0,
          // Speed down
          static_cast<int>(plan_start_d / LANE_WIDTH) * LANE_WIDTH + LANE_WIDTH / 2.0,
          // Speed up
          static_cast<int>(plan_start_d / LANE_WIDTH) * LANE_WIDTH + LANE_WIDTH / 2.0
     };

     // Jerk Minimization Coefficient
     // Simply use diff s as velocity
     // Simply use 0 as accel (no need to control speed if it's not neccesary)
     vector<double> start_s = {
          plan_start_s,
          0.0,
          0.0
     };
     if (path_queue.size() >= 2) {
          double ds = std::pow(path_queue[path_queue.size() - 1][0]
               - path_queue[path_queue.size() - 2][0], 2.0);
          double dd = std::pow(path_queue[path_queue.size() - 1][1]
               - path_queue[path_queue.size() - 2][1], 2.0);
          start_s[1] = std::sqrt(ds + dd) / UNIT_TIME;
     } else if (this->previous_path_x.size() > 0) {
          start_s[1] = this->NORMAL_SPEED;
     }

     // Use the following number as velocity (normal case)
     // 22.352 m/s (50MPH) / 1000 * 20(20 ms) * 80%(buffer) = around 0.35
     vector<double> no_speed_change_end_s = {
          this->map_waypoints_s[next_waypoint_ids[NUM_WAYPOINTS_USED_FOR_PATH - 1]],
          this->NORMAL_SPEED,
          0.0
     };
     vector<double> speed_down_end_s = {
          this->map_waypoints_s[next_waypoint_ids[NUM_WAYPOINTS_USED_FOR_PATH - 1]],
          this->car_speed * this->MPH_TO_MS - this->NORMAL_SPEED / 3.0,
          0.0
     };
     vector<double> speed_up_end_s = {
          this->map_waypoints_s[next_waypoint_ids[NUM_WAYPOINTS_USED_FOR_PATH - 1]],
          this->car_speed * this->MPH_TO_MS + this->NORMAL_SPEED / 3.0,
          0.0
     };
     vector<vector<double>> end_s_list = {
          no_speed_change_end_s,
          no_speed_change_end_s,
          no_speed_change_end_s,
          speed_down_end_s,
          speed_up_end_s
     };

     vector<double> pre_new_x;
     vector<double> pre_new_y;
     double plan_start_x = this->car_x;
     double plan_start_y = this->car_y;
     // Get last path car yaw with radian
     double last_yaw = this->Degree2Radian(this->car_yaw);
     // Do I have have previous points
     if (this->previous_path_x.size() < 2) {
          double prev_car_x = car_x - cos(last_yaw);
          double prev_car_y = car_y - sin(last_yaw);

          pre_new_x.push_back(prev_car_x);
          pre_new_x.push_back(car_x);

          pre_new_y.push_back(prev_car_y);
          pre_new_y.push_back(car_y);
     } else {
          // Use the last two points.
          plan_start_x = this->previous_path_x[this->previous_path_x.size() - 1];
          plan_start_y = this->previous_path_y[this->previous_path_x.size() - 1];

          double ref_x_prev = previous_path_x[this->previous_path_x.size() - 2];
          double ref_y_prev = previous_path_y[this->previous_path_x.size() - 2];
          last_yaw = atan2(plan_start_y - ref_y_prev, plan_start_x - ref_x_prev);

          pre_new_x.push_back(ref_x_prev);
          pre_new_x.push_back(plan_start_x);

          pre_new_y.push_back(ref_y_prev);
          pre_new_y.push_back(plan_start_y);
     }

     vector<vector<vector<double>>> candidates;
     // Generate candidate path with spline interpolation
     for (int i = 0; i < NUM_ACTIONS; i++) {
          vector<double> end_s = end_s_list[i];

          // Ignore out-of-lane path
          if (new_end_d[i] < 0.0 || new_end_d[i] > NUM_LANES * LANE_WIDTH) { continue; }
          // Ignore irregular speed
          if (end_s[1] <= 5.0 || end_s[1] >= SPEED_LIMIT) { continue; }

          // TODO(zuqqhi2): Need to be calculated accurately
          double end_t = (end_s[0] - start_s[0]) / ((end_s[1] + start_s[1]) / 2.0);
          vector<double> coef_s = this->CalculateJerkMinimizingCoef(start_s, end_s, end_t);

          // These setting make a path smoother with spline curve
          // *---+
          //     +---->
          vector<double> new_d = {plan_start_d, plan_start_d, new_end_d[i], new_end_d[i]};

          // Transform to XY cord
          vector<double> new_x;
          vector<double> new_y;
          for (int i = 0; i < pre_new_x.size(); i++) {
               new_x.push_back(pre_new_x[i]);
               new_y.push_back(pre_new_y[i]);
          }
          for (int i = 0; i < new_s.size(); i++) {
               vector<double> xy = this->GetXYFromSD(new_s[i], new_d[i],
                    this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
               new_x.push_back(xy[0]);
               new_y.push_back(xy[1]);
          }

          // Making coordinates to local car coordinates.
          for (int i = 0; i < new_x.size(); i++) {
               double shift_x = new_x[i] - new_x[0];
               double shift_y = new_y[i] - new_y[0];

               new_x[i] = shift_x * cos(0 - last_yaw) - shift_y * sin(0 - last_yaw);
               new_y[i] = shift_x * sin(0 - last_yaw) + shift_y * cos(0 - last_yaw);
          }

          // Spline fitting
          tk::spline sp;
          sp.set_points(new_x, new_y);

          vector<vector<double>> new_path_xy;

          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = sp(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          double N = target_dist / (0.02 * end_s[1]);
          for (int i = 1; i <= N; i++) {
               double x_point = x_add_on + target_x / N;
               double y_point = sp(x_point);

               x_add_on = x_point;

               double x_ref = x_point;
               double y_ref = y_point;

               x_point = x_ref * cos(last_yaw) - y_ref * sin(last_yaw);
               y_point = x_ref * sin(last_yaw) + y_ref * cos(last_yaw);

               x_point += plan_start_x;
               y_point += plan_start_y;

               new_path_xy.push_back({x_point, y_point});
          }

          double t = UNIT_TIME;
          while (t < end_t) {
               double s = this->CalculatePolynomialResult(coef_s, t);
               vector<double> sd = {s, sp(s)};
               new_path_sd.push_back(sd);
               t += UNIT_TIME;
          }

          candidates.push_back(new_path_xy);
     }
     */

     return candidates;
}

vector<vector<double>> PathPlanner::ChooseAppropriatePath(
     const vector<vector<vector<double>>> &paths, const map<int, Vehicle> &vehicles) {
     // Calculate total costs by all registered cost functions
     double min_cost = 1e+6;
     vector<vector<double>> min_cost_path;
     for (int i = 0; i < paths.size(); i++) {
          vector<vector<double>> path = paths[i];

          double total_cost = 0.0;
          /*
          for (int j = 0; j < NUM_COST_FUNCTIONS; j++) {
               total_cost += this->cost_functions[j]->CalculateCost(path, vehicles);
          }
          */

          if (total_cost < min_cost) {
               min_cost = total_cost;
               min_cost_path = path;
          }

          // Debug
          // std::cout << std::fixed;
          // std::cout << std::setprecision(2);
          // std::cout << i << ": s = (" << path[0][0] << ", " << path[path.size()-1][0]
          //      << ", d = (" << path[0][1] << ", " << path[path.size()-1][1]
          //      << "), total_cost = " << total_cost << ", min_cost = " << min_cost << std::endl;
     }
     // std::cout << std::endl;

     // Store
     for (int i = 0; i < min_cost_path.size(); i++) {
          vector<double> xy = {min_cost_path[i][0], min_cost_path[i][1]};
          path_queue.push_back(xy);
     }

     return min_cost_path;
}

vector<vector<double>> PathPlanner::GetPlannedPath() {
     vector<vector<double>> result;

     for (int i = 0; i < this->path_queue.size(); i++) {
          result.push_back(this->path_queue[0]);
          this->path_queue.erase(this->path_queue.begin());
     }

     return result;
}


/**
 * Calculate the Jerk Minimizing Coefficient that connects the initial state
 * to the final state in time T.
 *
 * @param start - the vehicles start location given as a length three array
 *   corresponding to initial values of [s, s_dot, s_double_dot]
 * @param end - the desired end state for vehicle. Like "start" this is a
 *   length three array.
 * @param T - The duration, in seconds, over which this maneuver should occur.
 *
 * @output an array of length 6, each value corresponding to a coefficent in 
 *   the polynomial:
 *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
 *
 * EXAMPLE
 *   > JMT([0, 10, 0], [10, 10, 0], 1)
 *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 */
vector<double> PathPlanner::CalculateJerkMinimizingCoef(
     const vector<double> &start, const vector<double> &end, double T) {
    Eigen::Matrix3d A;
    Eigen::Vector3d b;

    double T2 = T * T;
    double T3 = T * T * T;
    double T4 = T * T * T * T;
    double T5 = T * T * T * T * T;

    A << T3,       T4,        T5,
         3.0 * T2, 4.0 * T3,  5.0 * T4,
         6.0 * T,  12.0 * T2, 20.0 * T3;

    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
         end[1] - (start[1] + start[2] * T),
         end[2] - start[2];

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    return {start[0], start[1], 0.5 * start[2], x[0], x[1], x[2]};
}
