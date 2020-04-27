#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_

#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "vehicle.h"
#include "cost_function.h"

using std::vector;
using std::map;

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathPlanner {
 private:
    static const int NUM_COST_FUNCTIONS = 8;

    const double MAX_JERK = 10.0;  // m/s/s/s
    const double EXPECTED_JERK_IN_ONE_SEC = 2.0;  // m/s/s
    const double MAX_ACCEL = 10.0;  // m/s/s
    const double EXPECTED_ACC_IN_ONE_SEC = 1.0;  // m/s
    const vector<double> SIGMA_S = {10.0, 4.0, 2.0};
    const vector<double> SIGMA_D = {1.0, 1.0, 1.0};
    const double VEHICLE_RADIUS = 2.0;  // model vehicle as circle (prev=1.5)
    const double LANE_LEFT_LIMIT = 0.0;
    const double LANE_RIGHT_LIMIT = 12.0;  // Each lane is 4 m wide and there are 3 lanes

    // Each cost funtion's weight
    const double COST_WEIGHT_MAX_JERK = 10.0;  // Important
    const double COST_WEIGHT_COLLISION = 10.0;  // Important
    const double COST_WEIGHT_OUT_OF_LANE = 10.0;  // Important
    const double COST_WEIGHT_MAX_ACCEL = 10.0;  // Important
    const double COST_WEIGHT_GOAL_ARRIVE_TIME = 2.0;  // a little important
    const double COST_WEIGHT_TOTAL_JERK = 1.0;
    const double COST_WEIGHT_TOTAL_ACCEL = 1.0;
    const double COST_WEIGHT_SD_STATE_DIFF = 0.0;

    // Cost function set
    CostFunction *cost_functions[NUM_COST_FUNCTIONS];


    // Calculate differentiate
    vector<double> Differentiate(const vector<double> &x);

    // Calculate logit
    double Logistic(double x);

 public:
    // Constructor
    PathPlanner();
    // Destructor
    virtual ~PathPlanner() {}

    /**
     * Calculate the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in time T.
     */
    vector<double> CalculateJerkMinimizingCoef(
        const vector<double> &start, const vector<double> &end, double T);

    /**
     * Calculate trajectory's cost
     */
    double CalculateCost(const vector<double> &s, const vector<double> &d,
        const vector<double> &target_vehicle_state, const map<int, Vehicle> &vehicles,
        int num_div, double goal_t, double goal_s);

    // Calculate Polynomial Equation Result
    // s(t) = s_i + dot s_i * t + dot dot s_i / 2 * t^2
    //        + alpha_3 * t^3 + alpha_4 * t^4 + alpha_5 * t^5
    double CalculateEqRes(const vector<double> &x, double t);
};

#endif  // SRC_PATH_PLANNER_H_
