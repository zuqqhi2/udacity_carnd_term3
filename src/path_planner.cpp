#include <algorithm>
#include "path_planner.h"

// Setup cost function set with weight, min/max value for 0-1 scaling
PathPlanner::PathPlanner(const vector<double> &map_waypoints_x,
     const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s)
     : map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
     map_waypoints_s(map_waypoints_s) {
     cost_functions[0] = new MaxJerkCostFunction(COST_WEIGHT_MAX_JERK, MAX_JERK);
     cost_functions[1] = new CollisionCostFunction(COST_WEIGHT_COLLISION, VEHICLE_RADIUS);
     cost_functions[2] = new OutOfLaneCostFunction(COST_WEIGHT_OUT_OF_LANE,
          LANE_LEFT_LIMIT, LANE_RIGHT_LIMIT, VEHICLE_RADIUS, LANE_CENTERS);
     cost_functions[3] = new MaxAccelCostFunction(COST_WEIGHT_MAX_ACCEL, MAX_ACCEL);
     cost_functions[4] = new GoalArriveTimeCostFunction(COST_WEIGHT_GOAL_ARRIVE_TIME);
     cost_functions[5] = new TotalJerkCostFunction(
          COST_WEIGHT_TOTAL_JERK, EXPECTED_JERK_IN_ONE_SEC);
     cost_functions[6] = new TotalAccelCostFunction(
          COST_WEIGHT_TOTAL_ACCEL, EXPECTED_ACC_IN_ONE_SEC);
     cost_functions[7] = new DiffSDStateCostFunction(COST_WEIGHT_SD_STATE_DIFF);
     cost_functions[8] = new VehicleBufferCostFunction(COST_WEIGHT_VEHICLE_BUFFER, VEHICLE_RADIUS);
}

vector<double> PathPlanner::Differentiate(const vector<double> &x) {
     vector<double> result;
     for (int i = 1; i < x.size(); i++) {
          result.push_back((i + 1.0) * x[i]);
     }

     return result;
}

// Calculate logit
double PathPlanner::Logistic(double x) {
     return 2.0 / (1.0 + std::exp(-x)) - 1.0;
}

double PathPlanner::CalculateEqRes(const vector<double> &x, double t) {
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
vector<vector<vector<double>>> PathPlanner::GenerateCandidatePaths(int next_waypoint_id) {
     // Get some next way points
     int next_waypoint_ids[NUM_WAYPOINTS_USED_FOR_PATH];

     // Succeeding way points
     next_waypoint_ids[0] = next_waypoint_id;
     for (int i = 1; i < NUM_WAYPOINTS_USED_FOR_PATH; i++) {
          next_waypoint_ids[i] = (next_waypoint_ids[i - 1] + 1) % map_waypoints_x.size();
     }

     double plan_start_s = car_s;
     double plan_start_d = car_d;
     if (this->previous_path_x.size() == 0) {
          end_path_s = car_s;
          end_path_d = car_d;
     }

     vector<double> new_s = {plan_start_s};
     for (int i = 0; i < NUM_WAYPOINTS_USED_FOR_PATH; i++) {
          new_s.push_back(this->map_waypoints_s[next_waypoint_ids[i]]);
     }

     double new_end_d[NUM_ACTIONS] = {
          static_cast<int>((plan_start_d / LANE_WIDTH) - 1) * LANE_WIDTH + LANE_WIDTH / 2.0,
          static_cast<int>(plan_start_d / LANE_WIDTH) * LANE_WIDTH + LANE_WIDTH / 2.0,
          static_cast<int>((plan_start_d / LANE_WIDTH) + 1) * LANE_WIDTH + LANE_WIDTH / 2.0
     };

     // Generate candidate path with spline interpolation
     vector<vector<vector<double>>> candidates;
     for (int i = 0; i < NUM_ACTIONS; i++) {
          // These setting make a path smoother with spline curve
          // *---+
          //     +---->
          vector<double> new_d = {plan_start_d, plan_start_d, new_end_d[i], new_end_d[i]};

          // Ignore out-of-lane path
          if (new_d[1] < 0.0 || new_d[1] > NUM_LANES * LANE_WIDTH
               || new_d[2] < 0 || new_d[2] > NUM_LANES * LANE_WIDTH) { continue; }

          // Spline fitting
          tk::spline sp;
          sp.set_points(new_s, new_d);

          vector<vector<double>> new_path_sd;
          for (int i = 0; i < NUM_INTERPOLATION; i++) {
               double s = plan_start_s
                    + (this->map_waypoints_s[next_waypoint_ids[NUM_WAYPOINTS_USED_FOR_PATH - 1]]
                    - plan_start_s) / NUM_INTERPOLATION * i;

               vector<double> sd = {s, sp(s)};
               new_path_sd.push_back(sd);
          }

          candidates.push_back(new_path_sd);
     }

     return candidates;
}

vector<vector<double>> PathPlanner::ChooseAppropriatePath(
     const vector<vector<vector<double>>> &paths) {
     // Calculate total costs by all registered cost functions
     // vector<double> mycar_sd = {s[0], s[1], 2.0 * s[2], d[0], d[1], 2.0 * d[2]};

     double min_cost = 1e+6;
     vector<vector<double>> min_cost_path;
     /*
     for (int i = 0; i < this->paths.size(); i++) {
          double total_cost = 0.0;
          for (int j = 0; j < NUM_COST_FUNCTIONS; j++) {
               total_cost += this->cost_functions[j]->CalculateCost(
                    mycar_sd, target_vechicle_state, s, d, vehicles, num_div, goal_t, goal_s);
          }

          if (total_cost < min_cost) {
               min_cost = total_cost;
               min_cost_path = paths[i];
          }
     }
     */
     min_cost_path = paths[0];

     // Store
     for (int i = 0; i < min_cost_path.size(); i++) {
          vector<double> sd = {min_cost_path[i][0], min_cost_path[i][1]};
          path_queue.push_back(sd);
     }

     return min_cost_path;
}

vector<vector<double>> PathPlanner::GetPlannedPath(int num_points) {
     vector<vector<double>> result;

     int num = std::min(num_points, static_cast<int>(this->path_queue.size()));
     for (int i = 0; i < num_points; i++) {
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

// Calculate total cost of a given trajectory
double PathPlanner::CalculateCost(const vector<double> &s, const vector<double> &d,
     const vector<double> &target_vechicle_state, const map<int, Vehicle> &vehicles,
     int num_div, double goal_t, double goal_s) {
     // Calculate total costs by all registered cost functions
     vector<double> mycar_sd = {s[0], s[1], 2.0 * s[2], d[0], d[1], 2.0 * d[2]};
     double total_cost = 0.0;
     for (int i = 0; i < NUM_COST_FUNCTIONS; i++) {
          total_cost += this->cost_functions[i]->CalculateCost(
               mycar_sd, target_vechicle_state, s, d, vehicles, num_div, goal_t, goal_s);
     }

     return total_cost;
}
