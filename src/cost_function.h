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
    // Calculates the derivative of a polynomial and returns the corresponding coefficients.
    vector<double> Differentiate(const vector<double> &x);

    // A function that returns a value between 0 and 1 for x in the
    // range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    // Useful for cost functions.
    double Logistic(double x);

    // Calculate the polynomial equation result
    // s(t) = s_i + dot s_i * t + dot dot s_i / 2 * t^2
    //        + alpha_3 * t^3 + alpha_4 * t^4 + alpha_5 * t^5
    double CalculatePolynomialResult(const vector<double> &x, double t);

 public:
    double weight;  // Importance of this cost

    CostFunction() {
        weight = 0;
    }
    explicit CostFunction(double weight) {
        this->weight = weight;
    }

    ~CostFunction() {}

    // Calculate a specific cost between 0 ~ 1
    virtual double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) = 0;
};


// Calculate d and vs difference cost between target and my vehicle
class DiffSDStateCostFunction : public CostFunction {
 public:
    using CostFunction::CostFunction;

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate d and vs difference cost between target and my vehicle
class MaxJerkCostFunction : public CostFunction {
 private:
    double max_jerk;

 public:
    using CostFunction::CostFunction;
    MaxJerkCostFunction(double weight, double max_jerk) : CostFunction(weight) {
        this->max_jerk = max_jerk;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate collision cost with another vehicle
class CollisionCostFunction : public CostFunction {
 private:
    double vehicle_radius;

 public:
    using CostFunction::CostFunction;
    CollisionCostFunction(double weight, double vehicle_radius) : CostFunction(weight) {
        this->vehicle_radius = vehicle_radius;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate out of lane cost
class OutOfLaneCostFunction : public CostFunction {
 private:
    double lane_left_limit;
    double lane_right_limit;
    double vehicle_radius;

 public:
    using CostFunction::CostFunction;
    OutOfLaneCostFunction(double weight, double lane_left_limit,
        double lane_right_limit, double vehicle_radius) : CostFunction(weight) {
        this->lane_left_limit = lane_left_limit;
        this->lane_right_limit = lane_right_limit;
        this->vehicle_radius = vehicle_radius;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate goal arrive time cost(how far from current position)
class GoalArriveTimeCostFunction : public CostFunction {
 public:
    using CostFunction::CostFunction;

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate total jerk cost to make a path smooth
class TotalJerkCostFunction : public CostFunction {
 private:
    double expected_jerk_in_one_sec;

 public:
    using CostFunction::CostFunction;
    TotalJerkCostFunction(double weight, double expected_jerk_in_one_sec) : CostFunction(weight) {
        this->expected_jerk_in_one_sec = expected_jerk_in_one_sec;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate max accel cost to avoid using accel beyond vehicle capacity
class MaxAccelCostFunction : public CostFunction {
 private:
    double max_accel;

 public:
    using CostFunction::CostFunction;
    MaxAccelCostFunction(double weight, double max_accel) : CostFunction(weight) {
        this->max_accel = max_accel;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

// Calculate total accel cost to make a path smooth
class TotalAccelCostFunction : public CostFunction {
 private:
    double expected_accel_in_one_sec;

 public:
    using CostFunction::CostFunction;
    TotalAccelCostFunction(double weight, double expected_accel_in_one_sec) : CostFunction(weight) {
        this->expected_accel_in_one_sec = expected_accel_in_one_sec;
    }

    double CalculateCost(const vector<double> &my_sd, const vector<double> &target_sd,
        const vector<double> &coef_s, const vector<double> &coef_d,
        const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) override;
};

#endif  // SRC_COST_FUNCTION_H_
