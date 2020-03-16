#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>

#include "catch.hpp"
#include "../src/path_planner.h"


SCENARIO("PathPlanner can generate optimized trajectories", "[path_planner]") {

    GIVEN("A PathPlanner") {
        PathPlanner pp;

        WHEN("Simple move forward action start, end point and duration are given") {
            vector<double> start = {0, 10, 0};
            vector<double> end = {10, 10, 0};
            double T = 1;
            vector<double> jmt = pp.JerkMinimizingTrajectory(start, end, T);

            THEN("Minimum Jerk move forward trakectory is generated") {
                vector<double> answer = {0.0, 10.0, 0.0, 0.0, 0.0, 0.0};
                REQUIRE(jmt.size() == answer.size());

                for (int i = 0; i < jmt.size(); i++) {
                    double diff = jmt[i] - answer[i];
                    REQUIRE(abs(diff) < 0.01);
                }
            }
        }

    }

}