#include <algorithm>
#include "path_planner.h"

// Setup cost function set with weight, min/max value for 0-1 scaling
PathPlanner::PathPlanner() {
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

/**
 *
 */
double PathPlanner::CalculateCost(const vector<double> &s, const vector<double> &d,
     const vector<double> &target_vechicle_state, const map<int, Vehicle> &vehicles,
     int num_div, double goal_t, double goal_s) {
     // Calculate collision cost
     double closest = 1e+6;
     for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
          Vehicle v = item->second;
          double v_closest = 1e+6;

          for (int j = 0; j < num_div; j++) {
               double t = j / 100.0 * goal_t;
               double cur_s = this->CalculateEqRes(s, t);
               double cur_d = this->CalculateEqRes(d, t);
               double target_s = v.s_state[0] + (v.s_state[1] * t) + v.s_state[2] * t * t / 2.0;
               double target_d = v.d_state[0] + (v.d_state[1] * t) + v.d_state[2] * t * t / 2.0;
               double dist = std::sqrt((cur_s - target_s) * (cur_s - target_s)
                    + (cur_d - target_d) * (cur_d - target_d));
               v_closest = std::min(v_closest, dist);
          }

          closest = std::min(closest, v_closest);
     }
     double cost_collision = 0.0;
     if (closest < 2.0 * this->VEHICLE_RADIUS) { cost_collision = 1.0; }
     cost_collision *= 10.0;

     // Calculate buffer cost
     double cost_buffer = this->Logistic(2.0 * this->VEHICLE_RADIUS / closest);

     vector<double> mycar_sd = {s[0], s[1], 2.0 * s[2], d[0], d[1], 2.0 * d[2]};

     double total_cost = 0.0;
     for (int i = 0; i < NUM_COST_FUNCTIONS; i++) {
          total_cost += this->cost_functions[i]->CalculateCost(
               mycar_sd, target_vechicle_state, s, d, vehicles, num_div, goal_t, goal_s);
     }

     return total_cost;
}
