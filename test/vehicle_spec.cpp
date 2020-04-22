#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/vehicle.h"


SCENARIO("Vehicle can estimate a position in a future from current states", "[vehicle]") {
    GIVEN("A Vehicle") {
        Vehicle v;

        WHEN("Simple move forward action's pos, velocity, accel and future time are given") {
            v.s_state[0] = 3.0;
            v.s_state[1] = 2.0;
            v.s_state[2] = 1.0;
            
            v.d_state[0] = 4.0;
            v.d_state[1] = 3.0;
            v.d_state[2] = 2.0;
            
            double t = 1.0;

            vector<double> actual = v.PredictSDStateAt(t);
            THEN("New s, d axis states at t=1 are estimated") {
                vector<double> expected = {5.5, 3.0, 1.0, 8.0, 5.0, 2.0};
                REQUIRE(actual.size() == expected.size());

                for (int i = 0; i < actual.size(); i++) {
                    double diff = abs(actual[i] - expected[i]);
                    REQUIRE(diff < 0.01);
                }
            }
        }
    }
}
