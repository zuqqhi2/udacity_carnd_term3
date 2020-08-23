#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_

#include <cmath>
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
    static const int NUM_COST_FUNCTIONS = 3;

    const double VEHICLE_RADIUS = 1.0;  // model vehicle as circle (prev=1.5)
    const vector<double> LANE_CENTERS = {2.0, 6.0, 10.0};
    const int NUM_LANES = 3;
    const double UNIT_TIME = 0.02;

    const double MAX_VELOCITY = 49.5;
    const double VELOCITY_STEP = 0.224;
    const double MIN_LANE_CHANGE_VELOCITY_DOWN_RATE = 0.6;
    const double MAX_LANE_CHANGE_VELOCITY_DOWN_RATE = 0.8;
    const int NUM_FUTURE_REFERENCE_PATH_POINTS = 3;
    const double MAX_FUTURE_REFERENCE_S = 30.0;

    // Each cost funtion's weight
    const double COST_WEIGHT_COLLISION = 10.0;  // Important
    const double COST_WEIGHT_VEHICLE_BUFFER = 2.0;
    const double COST_WEIGHT_D_STATE_DIFF = 2.0;

    // States
    const int STATE_NORMAL = 0;
    const int STATE_PREPARE_LANE_CHANGE = 1;
    const int STATE_LANE_CHANGE = 2;

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

    // Another car info
    map<int, Vehicle> vehicles;

    // Cost function set
    CostFunction *cost_functions[NUM_COST_FUNCTIONS];

    // Calculate current lane id
    int GetLaneId(double d) { return static_cast<int>(d / this->LANE_WIDTH); }
    // Calculate given lane center
    double GetLaneCenter(int laneId) { return this->LANE_WIDTH * laneId + this->LANE_WIDTH / 2.0; }

    // State change
    void UpdateState();

 public:
    // Const values required to be shared with another object
    const double LANE_WIDTH = 4.0;

    // Current velocity
    double cur_velocity;
    int end_path_lane;
    int end_path_state;
    int future_target_lane = 1;

    // Constructor
    PathPlanner(): cur_velocity(0.0), end_path_state(STATE_NORMAL) {}
    PathPlanner(const vector<double> &map_waypoints_x,
        const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s);
    // Destructor
    virtual ~PathPlanner() {}

    // Get latest my car info
    void UpdateCarInfo(double x, double y, double s, double d, double yaw,
        double speed, const vector<double> &prev_path_x, const vector<double> &prev_path_y,
        double end_path_s, double end_path_d, const map<int, Vehicle> &vehicles);

    // Generate previous path
    vector<vector<double>> GeneratePreviousPath(double (*deg2rad)(double));

    // Generate best path
    vector<vector<double>> GenerateFuturePoints(const double ref_x, const double ref_y,
        const double ref_yaw, const vector<vector<double>> &pts, vector<double> (*getXY)(double,
        double, const vector<double>&, const vector<double>&, const vector<double>&));

    // Generate path
    vector<vector<double>> GenerateSmoothPath(
        const vector<vector<double>> &ref_pts, double ref_x, double ref_y, double ref_yaw);

    // Generate best path
    vector<vector<double>> GenerateBestPath(double (*deg2rad)(double),
        vector<double> (*getXY)(double, double,
        const vector<double>&, const vector<double>&, const vector<double>&));
};

#endif  // SRC_PATH_PLANNER_H_
