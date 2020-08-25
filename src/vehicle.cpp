#include "vehicle.h"

Vehicle::Vehicle() : id(-1), lane_width(0) {
    this->x = 0.0;
    this->vx = 0.0;
    this->y = 0.0;
    this->vy = 0.0;
    this->s = 0.0;
    this->d = 0.0;
}

// Initialize by given parameters
Vehicle::Vehicle(int id, int lane_width, const double x, const double vx, const double y,
    const double vy, const double s, const double d) : id(id), lane_width(lane_width) {
    this->UpdateState(x, vx, y, vy, s, d);
}

// Get current lane id
int Vehicle::GetLaneId() {
    if (this->d >= 0) {
        return static_cast<int>(this->d / this->lane_width);
    } else {
        return static_cast<int>(this->d / this->lane_width) - 1;
    }
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
}

// Estimate future(at given t) s-axis position
double Vehicle::PredictSPosAt(const double t) {
    return this->s + this->CalcSpeed() * t;
}
