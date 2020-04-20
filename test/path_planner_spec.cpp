#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/path_planner.h"


SCENARIO("PathPlanner can generate optimized trajectories", "[path_planner]") {
    GIVEN("A PathPlanner") {
        PathPlanner pp;

        WHEN("Simple move forward action start, end point and duration are given") {
            vector<double> start = {0, 10, 0};
            vector<double> end = {10, 10, 0};
            double T = 1;
            vector<double> jmt = pp.CalculateJerkMinimizingCoef(start, end, T);

            THEN("Minimum Jerk move forward trakectory is generated") {
                vector<double> answer = {0.0, 10.0, 0.0, 0.0, 0.0, 0.0};
                REQUIRE(jmt.size() == answer.size());

                for (int i = 0; i < jmt.size(); i++) {
                    double diff = jmt[i] - answer[i];
                    REQUIRE(abs(diff) < 0.01);
                }
            }
        }

        WHEN("Position of x axis and velocity are given") {
            vector<double> x_state = {10.0, 5.0};
            double T = 3.0;

            double new_x = pp.CalculateEqRes(x_state, T);
            THEN("Future x axis position at t=3 is estimated") {
                double expected = 10.0 + 5.0 * T;
                REQUIRE(abs(new_x - expected) < 0.01);
            }
        }
    }
}
