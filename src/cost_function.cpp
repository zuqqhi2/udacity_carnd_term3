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

/* DiffSDStateCostFunction */
double DiffSDStateCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    double cost = 0.0;

    // Calculate d diff cost
    vector<double> d_dot = this->Differentiate(coef_d);
    vector<double> d_dot_dot = this->Differentiate(d_dot);

    vector<double> D = {
        this->CalculatePolynomialResult(coef_d, end_t),
        this->CalculatePolynomialResult(d_dot, end_t),
        this->CalculatePolynomialResult(d_dot_dot, end_t)
    };

    vector<double> d_targets = {target_sd[3], target_sd[4], target_sd[5]};

    double cost_d_diff = 0.0;
    for (int i = 0; i < D.size(); i++) {
        double diff = std::abs(D[i] - d_targets[i]);
        // cost_d_diff += this->Logistic(diff / this->SIGMA_D[i]);
        cost_d_diff += this->Logistic(diff);
    }

    // Calculate s diff cost
    vector<double> s_dot = this->Differentiate(coef_s);
    vector<double> s_dot_dot = this->Differentiate(s_dot);

    vector<double> S = {
        this->CalculatePolynomialResult(coef_s, end_t),
        this->CalculatePolynomialResult(s_dot, end_t),
        this->CalculatePolynomialResult(s_dot_dot, end_t)
    };

    vector<double> s_targets = {target_sd[0], target_sd[1], target_sd[2]};

    double cost_s_diff = 0.0;
    for (int i = 0; i < S.size(); i++) {
        double diff = std::abs(S[i] - s_targets[i]);
        // cost_s_diff += this->Logistic(diff / this->SIGMA_S[i]);
        cost_s_diff += this->Logistic(diff);
    }

    return this->weight * this->Logistic(cost_d_diff + cost_s_diff);
}

/* MaxJerkCostFunction */
double MaxJerkCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    vector<double> s_dot = this->Differentiate(coef_s);
    vector<double> s_dot_dot = this->Differentiate(s_dot);
    vector<double> jerk = this->Differentiate(s_dot_dot);

    double max_j = -1e+6;
    for (int i = 0; i < num_div; i++) {
        double t = end_t / static_cast<double>(num_div) * i;
        double cur_jerk = std::abs(this->CalculatePolynomialResult(jerk, t));
        max_j = std::max(max_j, cur_jerk);
    }

    // no need scaling because cost is already 0 or 1
    if (max_j > this->max_jerk) {
        return this->weight * 1.0;
    } else {
        return 0.0;
    }
}

/* CollisionCostFunction */
double CollisionCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    double closest = 1e+6;
    for (auto item = vehicles.begin(); item != vehicles.end(); item++) {
        Vehicle v = item->second;

        for (int j = 0; j < num_div; j++) {
            double t = j / 100.0 * end_t;
            double cur_s = this->CalculatePolynomialResult(coef_s, t);
            double cur_d = this->CalculatePolynomialResult(coef_d, t);
            double target_s = v.s_state[0] + (v.s_state[1] * t) + v.s_state[2] * t * t / 2.0;
            double target_d = v.d_state[0] + (v.d_state[1] * t) + v.d_state[2] * t * t / 2.0;
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

/* OutOfLaneCostFunction */
double OutOfLaneCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    double left_most = 1e+6;
    double right_most = -1e+6;
    int num_centers = this->lane_centers.size();
    double dist_from_center[num_centers];
    for (int i = 0; i < num_div; i++) {
        double t = end_t / static_cast<double>(num_div) * i;
        double cur_d = this->CalculatePolynomialResult(coef_d, t);
        left_most = std::min(left_most, cur_d);
        right_most = std::max(right_most, cur_d);

        for (int j = 0; j < num_centers; j++) {
            dist_from_center[j] += std::abs(this->lane_centers[i] - cur_d);
        }
    }

    // If it's out of the right side, give max cost.
    // Other time, return dist from nearest lane center.
    if (left_most <= lane_left_limit + vehicle_radius / 2.0
        || right_most >= lane_right_limit - vehicle_radius / 2.0) {
        return this->weight * 1.0;
    } else {
        double min_dist = 1e+6;
        for (int j = 0; j < num_centers; j++) {
            min_dist = std::min(min_dist, dist_from_center[j]);
        }
        return this->weight * this->Logistic(min_dist);
    }
}

/* GoalArriveTimeCostFunction */
double GoalArriveTimeCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    double closest = 1e+6;
    for (int i = 0; i < num_div; i++) {
        double t = end_t / static_cast<double>(num_div) * i;
        double cur_s = std::abs(this->CalculatePolynomialResult(coef_s, t));

        double dist = goal_s - cur_s;  // dist shouldn't be never less than 0
        closest = std::min(closest, dist);
    }

    // How much the distance to the goal is decreased
    double diff = (goal_s - my_sd[0]) - closest;
    if (diff < 0) { diff = 0.0; }  // To avoid moving backward

    return this->weight * (1.0 - this->Logistic(diff));
}

/* TotalJerkCostFunction */
double TotalJerkCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    vector<double> s_dot = this->Differentiate(coef_s);
    vector<double> s_dot_dot = this->Differentiate(s_dot);
    vector<double> jerk = this->Differentiate(s_dot_dot);

    double dt = end_t / static_cast<double>(num_div);
    double total_jerk = 0.0;
    for (int i = 0; i < num_div; i++) {
        double t = dt * i;
        total_jerk += std::abs(this->CalculatePolynomialResult(jerk, t) * dt);
    }
    total_jerk /= static_cast<double>(num_div);

    return this->weight * this->Logistic(total_jerk / this->expected_jerk_in_one_sec);
}

/* MaxAccelCostFunction */
double MaxAccelCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    vector<double> s_dot = this->Differentiate(coef_s);
    vector<double> accel = this->Differentiate(s_dot);

    double max_a = -1e+6;
    for (int i = 0; i < num_div; i++) {
        double t = end_t / static_cast<double>(num_div) * i;
        double cur_accel = std::abs(this->CalculatePolynomialResult(accel, t));
        max_a = std::max(max_a, cur_accel);
    }

    // no need scaling because cost is already 0 or 1
    if (max_a > this->max_accel) {
        return this->weight * 1.0;
    } else {
        return 0.0;
    }
}

/* TotalAccelCostFunction */
double TotalAccelCostFunction::CalculateCost(const vector<double> &my_sd,
    const vector<double> &target_sd, const vector<double> &coef_s, const vector<double> &coef_d,
    const map<int, Vehicle> &vehicles, int num_div, double end_t, double goal_s) {
    vector<double> s_dot = this->Differentiate(coef_s);
    vector<double> accel = this->Differentiate(s_dot);

    double dt = end_t / static_cast<double>(num_div);
    double total_accel = 0.0;
    for (int i = 0; i < num_div; i++) {
        double t = dt * i;
        total_accel += std::abs(this->CalculatePolynomialResult(accel, t) * dt);
    }
    total_accel /= static_cast<double>(num_div);

    return this->weight * this->Logistic(total_accel / this->expected_accel_in_one_sec);
}
