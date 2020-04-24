#include "cost_function.h"

/*
vector<double> CostFunction::Differentiate(const vector<double> &x) {
    vector<double> result;
    for (int i = 1; i < x.size(); i++) {
        result.push_back((i + 1.0) * x[i]);
    }

    return result;
}

double CostFunction::Logistic(double x) {
    return 2.0 / (1.0 + std::exp(-x)) - 1.0;
}

double CostFunction::CalculatePolynomialResult(const vector<double> &x, double t) {
    double total = 0.0;
    for (int i = 0; i < x.size(); i++) {
        total += x[i] * std::pow(t, static_cast<double>(i));
    }

    return total;
}
*/


/* MaxJerkCostFunction */
/*
double MaxJerkCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, int num_div, double end_t) {
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
*/
