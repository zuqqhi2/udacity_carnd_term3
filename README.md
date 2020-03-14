# How to run

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
```

### Local

```sh
docker run --name udacity_carnd_term3_path_planning -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
git clone hhttps://github.com/zuqqhi2/udacity_carnd_term3_path_planning.git

# Compile & Run
cd udacity_carnd_term3_path_planning
mkdir build && cd build
cmake .. && make

# Unit Test by https://github.com/catchorg/Catch2
cd build
./${TestTarget} --reporter compact --success

# Lint
cpplint `find ./src -name *.cpp` 2>&1 | cat > cpplint.xml
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