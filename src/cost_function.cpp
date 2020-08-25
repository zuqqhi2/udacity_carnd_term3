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
            if (v.GetLaneId() != static_cast<int>(path[i][1] / this->lane_width)) { continue; }

            // Calculate when the car arrive this point
            double target_s = path[i][0] - path[i - 1][0];
            double target_d = path[i][1] - path[i - 1][1];
            double target_dist = sqrt(target_s * target_s + target_d * target_d);
            t += target_dist / (this->unit_time * cur_velocity / this->ms_2_mph);

            // Collision check
            double future_v_s = v.PredictSPosAt(
                (prev_size + static_cast<int>(t)) * this->unit_time);
            if (future_v_s > path[i][0]
                && (future_v_s - path[i][0]) < this->max_future_reference_s / path.size()) {
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
            if (v.GetLaneId() != static_cast<int>(path[i][1] / this->lane_width)) { continue; }

            // Calculate when the car arrive this point
            double target_s = path[i][0] - path[i - 1][0];
            double target_d = path[i][1] - path[i - 1][1];
            double target_dist = std::sqrt(target_s * target_s + target_d * target_d);
            t += target_dist / (this->unit_time * cur_velocity / this->ms_2_mph);

            // Calculate distance
            double future_v_s = v.PredictSPosAt(
                (prev_size + static_cast<int>(t)) * this->unit_time);
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