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
    virtual double CalculateCost(
        const vector<vector<double>> &path, const map<int, Vehicle> &vehicles)  = 0;
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

    double CalculateCost(
        const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) override;
};

// Calculate buffer cost to lessen colision chance
class VehicleBufferCostFunction : public CostFunction {
 private:
    double vehicle_radius;

 public:
    using CostFunction::CostFunction;
    VehicleBufferCostFunction(double weight, double vehicle_radius) : CostFunction(weight) {
        this->vehicle_radius = vehicle_radius;
    }

    double CalculateCost(
        const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) override;
};

// Calculate d difference cost between first point and end point of a path
class DiffDStateCostFunction : public CostFunction {
 public:
    using CostFunction::CostFunction;

    double CalculateCost(
        const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) override;
};

// Calculate goal arrive time cost(just check speed)
class GoalArriveTimeCostFunction : public CostFunction {
 private:
    double max_speed;

 public:
    using CostFunction::CostFunction;
    GoalArriveTimeCostFunction(double weight, double max_speed) : CostFunction(weight) {
        this->max_speed = max_speed;
    }

    double CalculateCost(
        const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) override;
};

#endif  // SRC_COST_FUNCTION_H_
