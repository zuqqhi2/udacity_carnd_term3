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
         * Calculate the Jerk Minimizing Trajectory that connects the initial state
         * to the final state in time T.
         */
        vector<double> JerkMinimizingTrajectory(vector<double> &start, vector<double> &end, double T);
};

#endif /* PATH_PLANNER_H */