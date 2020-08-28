#define CATCH_CONFIG_MAIN

#include <cmath>
#include <vector>
#include <map>

#include "catch.hpp"
#include "catch_reporter_sonarqube.hpp"

#include "../src/cost_function.h"

static const double epsilon = 0.01;
static const double UNIT_TIME = 0.02;
static const double MS_2_MPH = 2.24;
static const double MAX_FUTURE_REFERENCE_S = 30.0;
static const int LANE_WIDTH = 4;
static const double VEHICLE_RADIUS = 1.0;
static const double MAX_VELOCITY = 49.5;

SCENARIO("Cost functions can calculate costs for each situation", "[cost_function]") {
    GIVEN("A DiffDStateCostFunction") {
        DiffDStateCostFunction cf(1.0, UNIT_TIME, MS_2_MPH, MAX_FUTURE_REFERENCE_S, LANE_WIDTH);

        WHEN("path is given") {
            vector<vector<double>> path;
            path.push_back({0.0, 0.0});
            path.push_back({0.0, 1.1});

            map<int, Vehicle> vehicles;

            double actual = cf.CalculateCost(path, vehicles, 0, 0.0);
            THEN("id and lane_width are set") {
                double expected = 0.5;
                REQUIRE(std::abs(expected - actual) < 0.1);
            }
        }
    }

    GIVEN("A CollisionCostFunction") {
        CollisionCostFunction cf(1.0,
            UNIT_TIME, MS_2_MPH, MAX_FUTURE_REFERENCE_S, LANE_WIDTH, VEHICLE_RADIUS);

        WHEN("In collision situation") {
            vector<vector<double>> path;
            path.push_back({0.0, 6.0});
            path.push_back({0.0, 6.0});

            map<int, Vehicle> vehicles;
            vehicles[0] = Vehicle(0, LANE_WIDTH, 0.0, 1.0, 0.0, 0.0, 0.0, 6.0);

            double actual = cf.CalculateCost(path, vehicles, 1, 1.0);
            THEN("Cost function detect collision") {
                double expected = 1.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }

        WHEN("in not collision state") {
            vector<vector<double>> path;
            path.push_back({0.0, 6.0});
            path.push_back({1.0, 6.0});

            map<int, Vehicle> vehicles;
            vehicles[0] = Vehicle(0, LANE_WIDTH, 0.0, 1.0, 0.0, 0.0, 100.0, 6.0);

            double actual = cf.CalculateCost(path, vehicles, 1, 1.0);
            THEN("Cost function doesn't detect collision") {
                double expected = 0.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }
    }


    GIVEN("A VehicleBufferCostFunction") {
        VehicleBufferCostFunction cf(1.0,
            UNIT_TIME, MS_2_MPH, MAX_FUTURE_REFERENCE_S, LANE_WIDTH, VEHICLE_RADIUS);

        WHEN("In collision situation") {
            vector<vector<double>> path;
            path.push_back({0.0, 6.0});
            path.push_back({0.0, 6.0});

            map<int, Vehicle> vehicles;
            vehicles[0] = Vehicle(0, LANE_WIDTH, 0.0, 1.0, 0.0, 0.0, 0.0, 6.0);

            double actual = cf.CalculateCost(path, vehicles, 1, 1.0);
            THEN("Cost function returns highest cost") {
                double expected = 1.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }

        WHEN("in not collision state") {
            vector<vector<double>> path;
            path.push_back({0.0, 6.0});
            path.push_back({1.0, 6.0});

            map<int, Vehicle> vehicles;
            vehicles[0] = Vehicle(0, LANE_WIDTH, 0.0, 1.0, 0.0, 0.0, 100.0, 6.0);

            double actual = cf.CalculateCost(path, vehicles, 1, 1.0);
            THEN("Cost function returns lowest cost") {
                double expected = 0.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }
    }

    GIVEN("A SlowCostFunction") {
        SlowCostFunction cf(1.0,
            UNIT_TIME, MS_2_MPH, MAX_FUTURE_REFERENCE_S, LANE_WIDTH, MAX_VELOCITY);

        WHEN("max velocity is given") {
            vector<vector<double>> path;
            map<int, Vehicle> vehicles;

            double actual = cf.CalculateCost(path, vehicles, 0, 49.5);
            THEN("Cost is 0") {
                double expected = 0.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }

        WHEN("min velocity is given") {
            vector<vector<double>> path;
            map<int, Vehicle> vehicles;

            double actual = cf.CalculateCost(path, vehicles, 0, 0.0);
            THEN("Cost is 1") {
                double expected = 1.0;
                REQUIRE(std::abs(expected - actual) < epsilon);
            }
        }
    }
}
