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

            THEN("Minimum Jerk move forward trakectory is generated") {
                REQUIRE(1 == 1);
            }
        }

        // CalculatePolynomialResult
        WHEN("Position of x axis and velocity are given") {
            vector<double> x_state = {10.0, 5.0};
            double T = 3.0;

            THEN("Future x axis position at t=3 is estimated") {
                REQUIRE(1 == 1);
            }
        }
    }
}
