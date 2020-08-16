#include "cost_function.h"

double CostFunction::Logistic(double x) {
    return 2.0 / (1.0 + std::exp(-x)) - 1.0;
}

/* CollisionCostFunction */
double CollisionCostFunction::CalculateCost(
    const vector<vector<double>> &path,
    const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) {
    for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
        Vehicle v = item->second;

        double t = 0;
        for (int i = 1; i < path.size(); i++) {
            // Ignore a vehicle is in a different lane
            if (v.lane_id != static_cast<int>(path[i][1] / 4.0)) { continue; }

            // Calculate when the car arrive this point
            double target_s = path[i][0] - path[i - 1][0];
            double target_d = path[i][1] - path[i - 1][1];
            double target_dist = sqrt(target_s * target_s + target_d * target_d);
            t += target_dist / (0.02 * cur_velocity / 2.24);

            // Collision check
            double future_v_s = v.s_state[0] + (prev_size + static_cast<int>(t)) * 0.02 * v.speed;
            if (future_v_s > path[i][0] && (future_v_s - path[i][0]) < 30.0 / path.size()) {
                return this->weight * 1.0;
            }
        }
    }

    return 0.0;
}

/* VehicleBufferCostFunction */
double VehicleBufferCostFunction::CalculateCost(
    const vector<vector<double>> &path,
    const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) {
    double closest = 1e+6;
    for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
        Vehicle v = item->second;

        double t = 0;
        for (int i = 1; i < path.size(); i++) {
            // Ignore a vehicle is in a different lane
            if (v.lane_id != static_cast<int>(path[i][1] / 4.0)) { continue; }

            // Calculate when the car arrive this point
            double target_s = path[i][0] - path[i - 1][0];
            double target_d = path[i][1] - path[i - 1][1];
            double target_dist = std::sqrt(target_s * target_s + target_d * target_d);
            t += target_dist / (0.02 * cur_velocity / 2.24);

            // Calculate distance
            double future_v_s = v.s_state[0] + (prev_size + static_cast<int>(t)) * 0.02 * v.speed;
            double dist = std::abs(future_v_s - path[i][0]);
            closest = std::min(closest, dist);
        }
    }

    return this->weight * this->Logistic(2.0 * this->vehicle_radius / closest);
}

/* DiffDStateCostFunction */
double DiffDStateCostFunction::CalculateCost(
    const vector<vector<double>> &path,
    const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) {
    return this->weight * this->Logistic(std::abs(path[path.size() - 1][1] - path[0][1]));
}

/* GoalArriveTimeCostFunction */
// When the speed is close to the max speed, cost is the minimum
double GoalArriveTimeCostFunction::CalculateCost(
    const vector<vector<double>> &path,
    const map<int, Vehicle> &vehicles, const int prev_size, const double cur_velocity) {
    // return this->weight * this->Logistic(
    //    1.0 - std::abs(path[path.size() - 1][0] - path[0][0]) / (this->max_speed * path.size()));
    return 0.0;
}
