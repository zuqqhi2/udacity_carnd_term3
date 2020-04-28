#include "vehicle.h"

Vehicle::Vehicle() : id(-1), lane_id(0), speed(0.0) {
    for (int i = 0; i < 2; i++) {
        this->x_state[i] = 0.0;
        this->y_state[i] = 0.0;
    }

    for (int i = 0; i < 3; i++) {
        this->s_state[i] = 0.0;
        this->d_state[i] = 0.0;
    }
}

Vehicle::Vehicle(int id, const double (&x)[2], const double (&y)[2], double s, double d) : id(id) {
    for (int i = 0; i < 2; i++) {
        this->x_state[i] = x[i];
        this->y_state[i] = y[i];
    }

    for (int i = 0; i < 3; i++) {
        this->s_state[i] = 0.0;
        this->d_state[i] = 0.0;
    }
    this->s_state[0] = s;
    this->d_state[0] = d;

    // Detect lane id
    if (d >= 0) {
        this->lane_id = static_cast<int>(d / LANE_WIDTH) + 1;
    } else {
        this->lane_id = static_cast<int>(d / LANE_WIDTH) - 1;
    }

    // Calculate speed
    this->speed = std::sqrt(
        this->x_state[1] * this->x_state[1] + this->y_state[1] * this->y_state[1]);
}

Vehicle::Vehicle(int id, const double (&x)[2],
    const double (&y)[2], const double (&s)[3], const double (&d)[3]) : id(id) {
    for (int i = 0; i < 2; i++) {
        this->x_state[i] = x[i];
        this->y_state[i] = y[i];
    }

    for (int i = 0; i < 3; i++) {
        this->s_state[i] = s[i];
        this->d_state[i] = d[i];
    }

    // Detect lane id
    if (d >= 0) {
        this->lane_id = static_cast<int>(d[0] / LANE_WIDTH) + 1;
    } else {
        this->lane_id = static_cast<int>(d[0] / LANE_WIDTH) - 1;
    }

    // Calculate speed
    this->speed = std::sqrt(
        this->x_state[1] * this->x_state[1] + this->y_state[1] * this->y_state[1]);
}

// Estimate new s and d states from current states using following formula:
//   x_t = x_{t-1} + v_{t-1} * t + 1/2 * a_{t-1} * t^2
//   v_t = v_{t-1} + a_{t-1} * t
//   a_t = a_{t-1}
vector<double> Vehicle::PredictSDStateAt(double t) {
    // TODO(zuqqhi2): Use the following formula
    // return this->s_state[0] + prev_size * 0.02 * this->speed;

    return {
        this->s_state[0] + (this->s_state[1] * t) + this->s_state[2] * t * t / 2.0,
        this->s_state[1] + this->s_state[2] * t,
        this->s_state[2],
        this->d_state[0] + (this->d_state[1] * t) + this->d_state[2] * t * t / 2.0,
        this->d_state[1] + this->d_state[2] * t,
        this->d_state[2]
    };
}

// Predict future s-axis position with t
// If you want 20 ms base s position, you need to use 0.02 * x as t.
double Vehicle::PredictSPosAt(double t) {
    return this->s_state[0] + this->speed * t;
}

// Update x, y, s, d states
void Vehicle::UpdateState(const double (&x)[2], const double (&y)[2], double s, double d) {
    // Just copy for x, vx, y, vy
    for (int i = 0; i < 2; i++) {
        this->x_state[i] = x[i];
        this->y_state[i] = y[i];
    }

    // Update s and d
    double old_vs = this->s_state[1];
    this->s_state[1] = s - this->s_state[0];
    this->s_state[2] = this->s_state[1] - old_vs;
    this->s_state[0] = s;

    double old_vd = this->d_state[1];
    this->d_state[1] = d - this->d_state[0];
    this->d_state[2] = this->d_state[1] - old_vd;
    this->d_state[0] = d;

    // Detect lane id
    if (d >= 0) {
        this->lane_id = static_cast<int>(d / LANE_WIDTH) + 1;
    } else {
        this->lane_id = static_cast<int>(d / LANE_WIDTH) - 1;
    }

    // Calculate speed
    this->speed = std::sqrt(x[1] * x[1] + y[1] * y[1]);
}
