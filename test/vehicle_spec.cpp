#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/vehicle.h"

static const double epsilon = 0.01;

SCENARIO("Vehicle can estimate a position in a future", "[vehicle]") {
    GIVEN("A Vehicle with id and lane width") {
        Vehicle v(1, 4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        WHEN("At initialization") {
            int actual_id = v.id;
            int actual_lane_width = v.lane_width;
            
            THEN("id and lane_width are set") {
                int expected_id = 1;
                int expected_lane_width = 4;

                REQUIRE(actual_id == expected_id);
                REQUIRE(actual_lane_width == expected_lane_width);
            }
        }
    }

    GIVEN("A Vehicle") {
        Vehicle v;

        WHEN("Velocity is given") {
            v.UpdateState(0.0, 3.0, 0.0, 4.0, 0.0, 0.0);

            double actual = v.CalcSpeed();
            THEN("Speed is calculated") {
                double expected = 5.0;  // sqrt(vx^2 + vy^2) = sqrt(3.0 ^ 2 + 4.0 ^ 2) = 5.0

                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }

        WHEN("Lane width and d-axis position are given") {
            v.lane_width = 4;
            v.UpdateState(0.0, 0.0, 0.0, 0.0, 0.0, 6.6);

            int actual = v.GetLaneId();
            THEN("Lane id is detected") {
                int expected = 1;  // int(6.6 / 4) = int(1.6) = 1

                REQUIRE(actual == expected);
            }
        }

        WHEN("Velocity and future time are given") {
            v.UpdateState(0.0, 3.0, 0.0, 4.0, 0.0, 0.0);
            double t = 1.0;

            double actual = v.PredictSPosAt(t);
            THEN("New s axis position at t=1 is estimated") {
                double expected = 5.0;  // s0 + speed * t = 0.0 + 5.0 * 1.0 = 5.0

                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }
    }
}
