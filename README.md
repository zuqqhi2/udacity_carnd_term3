[![CircleCI](https://circleci.com/gh/zuqqhi2/udacity_carnd_term3_path_planning.svg?style=svg)](https://circleci.com/gh/zuqqhi2/udacity_carnd_term3_path_planning)
[![codecov](https://codecov.io/gh/zuqqhi2/udacity_carnd_term3_path_planning/branch/master/graph/badge.svg)](https://codecov.io/gh/zuqqhi2/udacity_carnd_term3_path_planning)
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=zuqqhi2_udacity_carnd_term3_path_planning&metric=alert_status)](https://sonarcloud.io/dashboard?id=zuqqhi2_udacity_carnd_term3_path_planning)

# Highway Driving Project

Controlling a vehicle to drive without any speed and collision violation with cost function and state machine.

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

# Result

Highway Driving control result Youtube link here

[![Highway Driving Result(Youtube)][resultss]](https://youtu.be/wUM6mPUW7Yc)

# How to controll
[statemachine]: ./images/statemachine.png "State Machine"

I'll share how to controll briefly here.

Path planning class(PathPlanner) does the followings every cycle.

1. Controling speed
2. When current state is not normal, just following current plan and move to step 5 otherwise step 3
3. Generating candidate paths
4. Calculate total cost for each candidate and pick up lowest cost path
5. Generating detail trajectory with spline curve fitting

## State Machine

![State Machine][statemachine]

## Cost Function

Cost functions are following.

|Cost Function Name|Summary                            | Weight |
|:----------------:|:---------------------------------:|:------:|
|Collision         |Collision will be happened or not  |10      |  
|Vehicle Buffer    |Distance from a closest vehicle    |2       |
|Diff D State      |Lane change will be happened or not|1       |

# How to compile & run

## Spline
https://kluge.in-chemnitz.de/opensource/spline/

## Compile

### Workspace

```sh
# Only 1st time at a day
# https://knowledge.udacity.com/questions/69207
rm /usr/lib/libuWS.so
cd /home/workspace/CarND-Path-Planning-Project
bash install-ubuntu.sh

# Compile
mkdir build && cd build
cmake .. && make

# FYI: Clean build
rm -rf ./build
# then do the same thing as Compile

# Comiple with SonarQube analysis
mkdir build && cd build
cmake ..
build-wrapper-linux-x86-64 --out-dir bw-output make clean all
mkdir test_execution_results
./PathPlannerSpec --reporter sonarqube -o test_execution_results/results.xml
mkdir coverages
gcovr CMakeFiles/PathPlannerSpec.dir/src --xml --keep -o coverages/sonar-cov.xml
cp CMakeFiles/PathPlannerSpec.dir/src/*path_planner.cpp.gcov coverages
cd ..
sonar-scanner -Dsonar.login=${SONAR_CLOUD_LOGIN}


# codecov.io
cd build
./PathPlannerSpec
mkdir coverages
lcov -c -b coverages -d CMakeFiles/PathPlannerSpec.dir/src/  -o coverage.info
lcov --remove coverage.info -o coverage_filtered.info '*/src/Eigen-3.3/*'
bash <(curl -s https://codecov.io/bash) -f coverage_filtered.info
```

### Local

```sh
docker run --name udacity_carnd_term3_path_planning -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
git clone https://github.com/zuqqhi2/udacity_carnd_term3_path_planning.git

# Compile & Run
cd udacity_carnd_term3_path_planning
mkdir build && cd build
cmake .. && make

# Unit Test by https://github.com/catchorg/Catch2
cd build
ctest
# For running one test 
# ./${TestTarget} --reporter compact --success
```

## Run

### Workspace

#### 1. Run the path planner

```sh
cd /home/workspace/CarND-Path-Planning-Project/build
./path_planning &
```

#### 2. Run the simulator

Under construction...

### Local

#### 1. Run the simulator

Latest simulator is the following.
https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2

Refered to https://knowledge.udacity.com/questions/78710

```sh
cd /path/to/unzipped_term3_sim_mac path
ls
# term3_sim.app
sudo chmod u+x term3_sim.app
chmod +x term3_sim.app/Contents/MacOS/term3_sim_mac
```

# Memo

Sensor data sample.
sensor_fusion data each index number means `[id, x, y, vx, vy, s, d]`.

```js
[
   "telemetry",
   {
      "x":939.1206,
      "y":1133.498,
      "yaw":6.264374,
      "speed":55.94247,
      "s":154.3344,
      "d":1.99995,
      "previous_path_x":[
         939.6176,
         940.1146,
         940.6117,
         941.1087,
         941.6057,
         942.1028,
         942.5998,
         943.0968,
         943.5939,
         944.0909,
         944.5879,
         945.085,
         945.582,
         946.079,
         946.576,
         947.0731,
         947.5701,
         948.0671,
         948.5641,
         949.0612,
         949.5582,
         950.0552,
         950.5523,
         951.0493,
         951.5463,
         952.0434,
         952.5404,
         953.0374,
         953.5345,
         954.0315,
         954.5285,
         955.0256,
         955.5226,
         956.0196,
         956.5167,
         957.0137,
         957.5107,
         958.0078,
         958.5048,
         959.0018,
         959.4988,
         959.9958,
         960.4929,
         960.9899,
         961.4869,
         961.9839,
         962.481,
         962.978,
         963.4751,
         963.9721,
         964.4691,
         964.9662,
         965.6838,
         966.1711,
         966.6584,
         967.1458,
         967.6331,
         968.1204,
         968.6077,
         969.095,
         969.5824,
         970.0697,
         970.557,
         971.0444,
         971.5317,
         972.019,
         972.5063,
         972.9937,
         973.481,
         973.9683,
         974.4556,
         974.943,
         975.4303,
         975.9177,
         976.405,
         976.8923,
         977.3796,
         977.8669,
         978.3542,
         978.8416,
         979.3289,
         979.8163,
         980.3036,
         980.7909,
         981.2783,
         981.7656,
         982.2529,
         982.7402,
         983.2275,
         983.7149,
         984.2022,
         984.6895,
         985.1769,
         985.6642,
         986.1515,
         986.6389
      ],
      "previous_path_y":[
         1133.552,
         1133.607,
         1133.661,
         1133.715,
         1133.77,
         1133.824,
         1133.879,
         1133.933,
         1133.987,
         1134.042,
         1134.096,
         1134.151,
         1134.205,
         1134.26,
         1134.314,
         1134.368,
         1134.423,
         1134.477,
         1134.531,
         1134.586,
         1134.64,
         1134.695,
         1134.749,
         1134.804,
         1134.858,
         1134.912,
         1134.967,
         1135.021,
         1135.076,
         1135.13,
         1135.184,
         1135.239,
         1135.293,
         1135.348,
         1135.402,
         1135.457,
         1135.511,
         1135.565,
         1135.62,
         1135.674,
         1135.729,
         1135.783,
         1135.837,
         1135.892,
         1135.946,
         1136.001,
         1136.055,
         1136.109,
         1136.164,
         1136.218,
         1136.273,
         1136.327,
         1136.475,
         1136.587,
         1136.699,
         1136.811,
         1136.922,
         1137.034,
         1137.146,
         1137.258,
         1137.37,
         1137.482,
         1137.594,
         1137.705,
         1137.817,
         1137.929,
         1138.041,
         1138.153,
         1138.265,
         1138.376,
         1138.488,
         1138.6,
         1138.712,
         1138.824,
         1138.936,
         1139.048,
         1139.16,
         1139.271,
         1139.383,
         1139.495,
         1139.607,
         1139.719,
         1139.831,
         1139.943,
         1140.054,
         1140.166,
         1140.278,
         1140.39,
         1140.502,
         1140.614,
         1140.725,
         1140.837,
         1140.949,
         1141.061,
         1141.173,
         1141.285
      ],
      "end_path_s":202.4179,
      "end_path_d":-0.5710588,
      "sensor_fusion":[
         [
            0,
            1044.806,
            1163.768,
            15.18749,
            6.181476,
            264.4855,
            1.959272
         ],
         [
            1,
            775.8,
            1425.2,
            0,
            0,
            6719.219,
            -280.1494
         ],
         [
            2,
            775.8,
            1429,
            0,
            0,
            6716.599,
            -282.9019
         ],
         [
            3,
            775.8,
            1432.9,
            0,
            0,
            6713.911,
            -285.7268
         ],
         [
            4,
            775.8,
            1436.3,
            0,
            0,
            6711.566,
            -288.1896
         ],
         [
            5,
            775.8,
            1441.7,
            0,
            0,
            6661.772,
            -291.7797
         ],
         [
            6,
            762.1,
            1421.6,
            0,
            0,
            6711.778,
            -268.0963
         ],
         [
            7,
            762.1,
            1425.2,
            0,
            0,
            6709.296,
            -270.7039
         ],
         [
            8,
            762.1,
            1429,
            0,
            0,
            6663.543,
            -273.1828
         ],
         [
            9,
            762.1,
            1432.9,
            0,
            0,
            6660.444,
            -275.5511
         ],
         [
            10,
            762.1,
            1436.3,
            0,
            0,
            6657.743,
            -277.6157
         ],
         [
            11,
            762.1,
            1441.7,
            0,
            0,
            6653.453,
            -280.8947
         ]
      ]
   }
]
```
