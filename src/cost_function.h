#ifndef SRC_COST_FUNCTION_H_
#define SRC_COST_FUNCTION_H_

#include <vector>
#include <cmath>
#include <algorithm>

using std::vector;

class CostFunction {
 protected:
    const double MAX_VAL_UPDATE_SCALE = 1.5;  // Multiply 1.5 when max_val is updated

    // Scale a cost between 0 ~ 1
    double TransformWithStandardScaling(double cost) {
        return (cost - this->min_val) / (this->max_val - this->min_val);
    }

    // Calculate differentiate
    vector<double> Differentiate(const vector<double> &x) {
        vector<double> result;
        for (int i = 1; i < x.size(); i++) {
            result.push_back((i + 1.0) * x[i]);
        }

        return result;
    }

    // Calculate logit
    double Logistic(double x) {
        return 2.0 / (1.0 + std::exp(-x)) - 1.0;
    }

    // Calculate the polynomial equation result
    // s(t) = s_i + dot s_i * t + dot dot s_i / 2 * t^2
    //        + alpha_3 * t^3 + alpha_4 * t^4 + alpha_5 * t^5
    double CalculatePolynomialResult(const vector<double> &x, double t) {
        double total = 0.0;
        for (int i = 0; i < x.size(); i++) {
            total += x[i] * std::pow(t, static_cast<double>(i));
        }

        return total;
    }

 public:
    double min_val, max_val;  // For 0~1 scaling
    double weight;  // Importance of this cost

    CostFunction() {
        weight = 0;
        min_val = 0;
        max_val = 0;
    }
    CostFunction(double weight, double min_val, double max_val) {
        this->weight = weight;
        this->min_val = min_val;
        this->max_val = max_val;
    }

    ~CostFunction() {}

    // Calculate a specific cost between 0 ~ 1
    virtual double CalculateCost(const vector<double> &my_sd,
        const vector<double> &target_sd, const vector<double> &coef_s,
        int num_div, double end_t) = 0;
};


// Calculate d and vs difference cost between target and my vehicle
class DiffSDStateCostFunction : public CostFunction {
 public:
    using CostFunction::CostFunction;

    double CalculateCost(const vector<double> &my_sd,
        const vector<double> &target_sd, const vector<double> &coef_s,
        int num_div, double end_t) override {
        double cost = 0.0;

        // d diff
        cost += std::abs(my_sd[3] - target_sd[3]);
        // vs diff
        cost += std::abs(my_sd[1] - target_sd[1]);

        // update max_val
        if (cost > this->max_val) {
            this->max_val = cost * this->MAX_VAL_UPDATE_SCALE;
        }

        return this->weight * this->TransformWithStandardScaling(cost);
    }
};

// Calculate d and vs difference cost between target and my vehicle
class MaxJerkCostFunction : public CostFunction {
 private:
    double max_jerk;

 public:
    using CostFunction::CostFunction;
    MaxJerkCostFunction(double weight, double min_val, double max_val, double max_jerk) {
        this->max_jerk = max_jerk;
        MaxJerkCostFunction(weight, min_val, max_val);
    }

    double CalculateCost(const vector<double> &my_sd,
        const vector<double> &target_sd, const vector<double> &coef_s,
        int num_div, double end_t) override {
        vector<double> s_dot = this->Differentiate(coef_s);
        vector<double> s_dot_dot = this->Differentiate(s_dot);
        vector<double> jerk = this->Differentiate(s_dot_dot);

        double max_j = -1e+6;
        for (int i = 0; i < num_div; i++) {
            double t = end_t / static_cast<double>(num_div) * i;
            double cur_jerk = std::abs(this->CalculatePolynomialResult(jerk, t));
            max_jerk = std::max(max_j, cur_jerk);
        }

        double cost_max_jerk = 0.0;
        if (max_j > this->max_jerk) { cost_max_jerk = 1.0; }

        return this->weight * cost_max_jerk;  // no need scaling because cost is already 0 or 1
    }
};

#endif  // SRC_COST_FUNCTION_H_
