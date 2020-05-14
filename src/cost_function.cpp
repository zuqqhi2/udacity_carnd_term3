#include "cost_function.h"

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

/* CollisionCostFunction */
double CollisionCostFunction::CalculateCost(
    const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) {
    double closest = 1e+6;
    for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
        Vehicle v = item->second;

        for (int j = 1; j <= path.size(); j++) {
            double t = j * 0.02;  // 20ms TODO(zuqqhi2): 0.02 should be const value
            double cur_s = path[j - 1][0];
            double cur_d = path[j - 1][1];
            double target_s = v.PredictSPosAt(t);
            double target_d = v.d_state[0];
            double dist = std::sqrt((cur_s - target_s) * (cur_s - target_s)
                + (cur_d - target_d) * (cur_d - target_d));

            closest = std::min(closest, dist);
        }
    }

    // no need scaling because cost is already 0 or 1
    if (closest < 2.0 * this->vehicle_radius) {
        return this->weight * 1.0;
    } else {
        return 0.0;
    }
}

/* VehicleBufferCostFunction */
double VehicleBufferCostFunction::CalculateCost(
    const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) {
    double closest = 1e+6;
    for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
        Vehicle v = item->second;

        for (int j = 1; j <= path.size(); j++) {
            double t = j * 0.02;  // 20ms TODO(zuqqhi2): 0.02 should be const value
            double cur_s = path[j - 1][0];
            double cur_d = path[j - 1][1];
            double target_s = v.PredictSPosAt(t);
            double target_d = v.d_state[0];
            double dist = std::sqrt((cur_s - target_s) * (cur_s - target_s)
                + (cur_d - target_d) * (cur_d - target_d));

            closest = std::min(closest, dist);
        }
    }

    return this->weight * this->Logistic(2.0 * this->vehicle_radius / closest);
}

/* DiffDStateCostFunction */
double DiffDStateCostFunction::CalculateCost(
    const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) {
    return this->weight * this->Logistic(std::abs(path[path.size() - 1][1] - path[0][1]));
}

/* GoalArriveTimeCostFunction */
double GoalArriveTimeCostFunction::CalculateCost(
    const vector<vector<double>> &path, const map<int, Vehicle> &vehicles) {
    return this->weight * this->Logistic(
        std::abs(path[path.size() - 1][0] - path[0][0]) / (this->max_speed * path.size()));
}
