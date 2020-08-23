#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <vector>
#include <cmath>

using std::vector;

class Vehicle {
 public:
    int id;  // Vehicle unique ID
    int lane_width;
    double x, vx;
    double y, vy;
    double s;
    double d;
    int lane_id;
    double speed;

    Vehicle();
    Vehicle(int id, int lane_width, const double x,
        const double vx, const double y, const double vy, const double s, const double d);
    virtual ~Vehicle() {}

    // Update vechile state
    void UpdateState(const double x,
        const double vx, const double y, const double vy, double s, double d);
    
    // Estimate future(at given t) s-axis position
    double PredictSPosAt(const double t);
};

#endif  // SRC_VEHICLE_H_
