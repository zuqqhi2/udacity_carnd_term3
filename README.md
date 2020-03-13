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

Under constructions...

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