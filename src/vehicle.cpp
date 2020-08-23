#include "vehicle.h"

Vehicle::Vehicle() : id(-1), lane_id(0), speed(0.0) {
    this->x = 0.0;
    this->vx = 0.0;
    this->y = 0.0;
    this->vy = 0.0;
    this->s = 0.0;
    this->d = 0.0;
}

// Initialize by given parameters
Vehicle::Vehicle(int id, int lane_width, const double x, const double vx,
    const double y, const double vy, const double s, const double d) : id(id), lane_width(lane_width) {
    this->UpdateState(x, vx, y, vy, s, d);
}

// Update state
void Vehicle::UpdateState(const double x,
        const double vx, const double y, const double vy, double s, double d) {
    // Just copy for x, vx, y, vy
    this->x = x;
    this->vx = vx;
    this->y = y;
    this->vy = vy;
    this->s = s;
    this->d = d;

    // Detect lane id
    if (this->d >= 0) {
        this->lane_id = static_cast<int>(this->d / this->lane_width);
    } else {
        this->lane_id = static_cast<int>(this->d / this->lane_width) - 1;
    }

    // Calculate speed
    this->speed = std::sqrt(this->vx * this->vx + this->vy * this->vy);
}

// Estimate future(at given t) s-axis position
double Vehicle::PredictSPosAt(const double t) {
    return this->s + this->speed * t;
}
