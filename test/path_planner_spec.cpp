#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>
#include <map>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/path_planner.h"

const double EPSILON = 1e-3;

double dummy_deg2rad(double deg) { return 0.0; }

SCENARIO("PathPlanner can generate optimized trajectories", "[path_planner]") {
    GIVEN("A PathPlanner") {
        vector<double> mw_x, mw_y, mw_s;
        PathPlanner pp(mw_x, mw_y, mw_s);

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

            vector<vector<double>> actual_path = pp.GeneratePreviousPath(dummy_deg2rad);
            THEN("previous path is generated") {
                vector<vector<double>> expected_path;
                expected_path.push_back({-1.0, 0.0});
                expected_path.push_back({0.0, 0.0});

                for (int i = 0; i < actual_path.size(); i++) {
                    REQUIRE(std::abs(expected_path[i][0] - actual_path[i][0]) < EPSILON);
                    REQUIRE(std::abs(expected_path[i][1] - actual_path[i][1]) < EPSILON);
                }
            }
        }
    }
}
