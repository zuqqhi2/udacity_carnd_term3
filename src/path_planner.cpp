#include "path_planner.h"

// Constructor
PathPlanner::PathPlanner() {}

// Destructor
PathPlanner::~PathPlanner() {}

// Main Function
/*
vector<double> PathPlanner::GenerateTrajectory(vector<double> &start_s, vector<double> &end_s, vector<double> &start_d, vector<double> &end_d, double yaw, double speed, double T, double goal_time) {
     vector<double> s_coef = this->JerkMinimizingTrajectory(start_s, end_s, goal_time);
     double new_s = this->CalculateTrajectoryEquation(s_coef, T + 0.01);

     vector<double> d_coef = this->JerkMinimizingTrajectory(start_d, end_d, goal_time);
     double new_d = this->CalculateTrajectoryEquation(d_coef, T + 0.01);

     return {new_s, new_d};
}
*/

double PathPlanner::CalculateTrajectoryEquation(vector<double> &coef, double t) {
     double result = 0.0;
     for (int i = 0; i < coef.size(); i++) {
          result += coef[i] * std::pow(t, i);
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
vector<double> PathPlanner::CalculateJerkMinimizingCoef(vector<double> &start, vector<double> &end, double T) {
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