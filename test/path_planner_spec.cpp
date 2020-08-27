#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>
#include <map>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/path_planner.h"


SCENARIO("PathPlanner can generate optimized trajectories", "[path_planner]") {
    GIVEN("A PathPlanner") {
        PathPlanner pp;

        WHEN("New car info is given") {
            vector<double> prev_path_x, prev_path_y;
            map<int, Vehicle> vehicles;

            pp.UpdateCarInfo(
                0.0, 0.0, 0.0, 6.0, 0.0, 0.0, prev_path_x, prev_path_y, 0.0, 6.0, vehicles);
            int actual_lane = pp.end_path_lane;
            int actual_state = pp.end_path_state;
            THEN("End path lane id and state are updated") {
                int expected_lane = 1;
                int expected_state = 0;

                REQUIRE(actual_lane == expected_lane);
                REQUIRE(actual_state == expected_state);
            }
        }
    }
}
