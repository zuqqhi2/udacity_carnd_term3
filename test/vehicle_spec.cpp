#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/vehicle.h"

static const double epsilon = 0.01;

SCENARIO("Vehicle can estimate a position in a future", "[vehicle]") {
    GIVEN("A Vehicle") {
        Vehicle v;

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
