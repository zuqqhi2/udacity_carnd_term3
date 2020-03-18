#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class PathPlanner {
    public:
        // Constructor
        PathPlanner();
        // Destructor
        virtual ~PathPlanner();

        /**
         * Return new s and d list
         */
        vector<double> GenerateTrajectory(double s, double d, double yaw, double speed);

        /**
         * s(t) = s_i + dot s_i * t + dot dot s_i / 2 * t^2 + alpha_3 * t^3 + alpha_4 * t^4 + alpha_5 * t^5  
         */ 
        double CalculateTrajectoryEquation(vector<double> coef);

        /**
         * Calculate the Jerk Minimizing Trajectory that connects the initial state
         * to the final state in time T.
         */
        vector<double> JerkMinimizingTrajectory(vector<double> &start, vector<double> &end, double T);
};

#endif /* PATH_PLANNER_H */