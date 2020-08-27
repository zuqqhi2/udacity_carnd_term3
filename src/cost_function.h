#ifndef SRC_COST_FUNCTION_H_
#define SRC_COST_FUNCTION_H_

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include "vehicle.h"

using std::vector;
using std::map;

class CostFunction {
 protected:
    // A function that returns a value between 0 and 1 for x in the
    // range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    // Useful for cost functions.
    double Logistic(double x);

 public:
    double weight;  // Importance of this cost
    double unit_time;
    double ms_2_mph;
    double max_future_reference_s;
    int lane_width;

    CostFunction() {
        weight = 0.0;
        unit_time = 0.0;
        ms_2_mph = 0.0;
        max_future_reference_s = 0.0;
        lane_width = 0;
    }
    explicit CostFunction(double weight, double unit_time,
        double ms_2_mph, double max_future_reference_s, int lane_width) {
        this->weight = weight;
        this->unit_time = unit_time;
        this->ms_2_mph = ms_2_mph;
        this->max_future_reference_s = max_future_reference_s;
        this->lane_width = lane_width;
    }

    ~CostFunction() {}

    // Calculate a specific cost between 0 ~ 1
    virtual double CalculateCost(
        const vector<vector<double>> &path,
        const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) = 0;
};

// Calculate collision cost with another vehicle
class CollisionCostFunction : public CostFunction {
 private:
    double vehicle_radius;

 public:
    using CostFunction::CostFunction;
    CollisionCostFunction(double weight, double unit_time, double ms_2_mph,
        double max_future_reference_s, int lane_width, double vehicle_radius)
        : CostFunction(weight, unit_time, ms_2_mph, max_future_reference_s, lane_width) {
        this->vehicle_radius = vehicle_radius;
    }

    double CalculateCost(
        const vector<vector<double>> &path,
        const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) override;
};

// Calculate buffer cost to lessen colision chance
class VehicleBufferCostFunction : public CostFunction {
 private:
    double vehicle_radius;

 public:
    using CostFunction::CostFunction;
    VehicleBufferCostFunction(double weight, double unit_time, double ms_2_mph,
        double max_future_reference_s, int lane_width, double vehicle_radius)
        : CostFunction(weight, unit_time, ms_2_mph, max_future_reference_s, lane_width) {
        this->vehicle_radius = vehicle_radius;
    }

    double CalculateCost(
        const vector<vector<double>> &path,
        const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) override;
};

// Calculate slow speed cost
class SlowCostFunction : public CostFunction {
 private:
    double max_velocity;

 public:
    using CostFunction::CostFunction;
    SlowCostFunction(double weight, double unit_time, double ms_2_mph,
        double max_future_reference_s, int lane_width, double max_velocity)
        : CostFunction(weight, unit_time, ms_2_mph, max_future_reference_s, lane_width) {
        this->max_velocity = max_velocity;
    }

    double CalculateCost(
        const vector<vector<double>> &path,
        const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) override;
};

// Calculate d difference cost between first point and end point of a path
class DiffDStateCostFunction : public CostFunction {
 public:
    using CostFunction::CostFunction;

    double CalculateCost(
        const vector<vector<double>> &path,
        const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) override;
};

#endif  // SRC_COST_FUNCTION_H_
