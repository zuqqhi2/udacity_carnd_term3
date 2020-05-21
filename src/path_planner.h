#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_

#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "spline.h"

#include "vehicle.h"
#include "cost_function.h"

using std::vector;
using std::map;

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathPlanner {
 private:
    static const int NUM_COST_FUNCTIONS = 5;

    const double SPEED_LIMIT = 22.352;
    const double MAX_SPEED = 0.44;  // 22.352 m/s (50MPH) / 0.02(20 ms) = around 0.44
    const double MAX_JERK = 10.0;  // m/s/s/s
    const double EXPECTED_JERK_IN_ONE_SEC = 2.0;  // m/s/s
    const double MAX_ACCEL = 10.0;  // m/s/s
    const double EXPECTED_ACC_IN_ONE_SEC = 1.0;  // m/s
    const double VEHICLE_RADIUS = 1.0;  // model vehicle as circle (prev=1.5)
    const double LANE_LEFT_LIMIT = 0.0;
    const double LANE_RIGHT_LIMIT = 12.0;  // Each lane is 4 m wide and there are 3 lanes
    const vector<double> LANE_CENTERS = {2.0, 6.0, 10.0};
    const double LANE_WIDTH = 4.0;
    const int NUM_LANES = 3;
    const int NUM_INTERPOLATION = 100;
    const double NORMAL_SPEED = 18.0;  // around 40 m/s
    const double MPH_TO_MS = 0.44704;  // Coefficient MPH to m/s
    const double UNIT_TIME = 0.02;

    // Each cost funtion's weight
    const double COST_WEIGHT_COLLISION = 10.0;  // Important
    const double COST_WEIGHT_VEHICLE_BUFFER = 2.0;
    const double COST_WEIGHT_D_STATE_DIFF = 2.0;
    const double COST_WEIGHT_GOAL_ARRIVE_TIME = 2.0;
    const double COST_WEIGHT_DIFF_SPEED = 1.0;  // Low importance

    // Way points
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_d;

    // Car info
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;

    // Cost function set
    CostFunction *cost_functions[NUM_COST_FUNCTIONS];

    // Calculate differentiate
    vector<double> Differentiate(const vector<double> &x);

    // Calculate logit
    double Logistic(double x);

 public:
    static const int NUM_WAYPOINTS_USED_FOR_PATH = 3;
    static const int NUM_QUEUE_PATH = 100;

    // Kind of behaviors
    //   1: Go Straight
    //   2: Move to left lane
    //   3: Move to right lane
    //   4: Slow down
    //   5: Speed up
    const int NUM_ACTIONS = 5;

    // Path history (s, d)
    vector<vector<double>> path_queue;

    // Constructor
    PathPlanner() {}
    PathPlanner(const vector<double> &map_waypoints_x,
        const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s);
    // Destructor
    virtual ~PathPlanner() {}

    // Get latest my car info
    void UpdateCarInfo(double x, double y, double s, double d, double yaw,
        double speed, const vector<double> &prev_path_x, const vector<double> &prev_path_y,
        double end_path_s, double end_path_d);

    // Generate candidates paths
    vector<vector<vector<double>>> GenerateCandidatePaths(int next_waypoint_id);

    // Choose appropriate path for current situation from candidates
    vector<vector<double>> ChooseAppropriatePath(
        const vector<vector<vector<double>>> &paths, const map<int, Vehicle> &vehicles);

    // Dequeue from palnned path queue
    vector<vector<double>> GetPlannedPath(int num_points);

    /**
     * Calculate the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in time T.
     */
    vector<double> CalculateJerkMinimizingCoef(
        const vector<double> &start, const vector<double> &end, double T);

    // Calculate the polynomial equation result
    // s(t) = s_i + dot s_i * t + dot dot s_i / 2 * t^2
    //        + alpha_3 * t^3 + alpha_4 * t^4 + alpha_5 * t^5
    double CalculatePolynomialResult(const vector<double> &x, double t);
};

#endif  // SRC_PATH_PLANNER_H_
