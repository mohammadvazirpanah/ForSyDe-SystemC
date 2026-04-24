# D*-Lite on TurtleBot3 — Setup & Run Guide

**Project:** Self-Aware CPS with SADF/ForSyDe-SystemC + ROS/Gazebo  
**Repository:** https://github.com/mohammadvazirpanah/ForSyDe-SystemC/tree/sadf-dstarlite-ros

---

## Prerequisites

- Ubuntu 20.04 (or compatible Linux)
- Docker installed
- X11 display server running

---

## Step 1 — Clone the Repositories

```bash
git clone https://github.com/mohammadvazirpanah/ForSyDe-SystemC.git \
    -b sadf-dstarlite-ros ~/ForSyDe-SystemC

git clone https://github.com/mohammadvazirpanah/forsyde-ros-gazebo-worlds.git \
    ~/catkin_ws/src/dstarlite_maze
```

---

## Step 2 — Install SystemC 2.3.3

```bash
wget https://www.accellera.org/images/downloads/standards/systemc/systemc-2.3.3.tar.gz
tar -xf systemc-2.3.3.tar.gz
cd systemc-2.3.3 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/deps/systemc
make -j$(nproc) && make install
```

---

## Step 3 — Create and Start Docker Container

```bash
xhost +local:

docker run -it --name ros_noetic \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/catkin_ws:/catkin_ws \
  -v ~/ForSyDe-SystemC:/forsyde \
  -v ~/deps:/deps \
  osrf/ros:noetic-desktop-full bash
```

---

## Step 4 — Inside Docker: Install Dependencies and Build

```bash
# Install TurtleBot3 packages
apt-get install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo

# Build the Gazebo world (catkin workspace)
source /opt/ros/noetic/setup.bash
cd /catkin_ws && catkin_make

# Build the ForSyDe-SystemC simulation
cd /forsyde/examples/sadf/dstarlite_3d
mkdir -p gen && make clean && make
```

---

## Step 5 — Recompile (after any code change)

If you modify any source file, recompile inside Docker:

```bash
docker exec ros_noetic bash -c \
  "cd /forsyde/examples/sadf/dstarlite_3d && make clean && make"
```

---

## Step 7 — Create the Named Pipe (once only)

Run this on the **host** (not inside Docker):

```bash
mkfifo ~/ForSyDe-SystemC/examples/sadf/dstarlite_3d/gen/self_report
```

---

## Step 8 — Run (3 Terminals)

> **Order matters:** Start Terminal 1 first and wait for Gazebo to load,
> then Terminal 2, then Terminal 3.

### Terminal 1 — Gazebo Simulation

```bash
xhost +local: && \
docker exec \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  ros_noetic bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source /catkin_ws/devel/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   roslaunch dstarlite_maze dstarlite_maze.launch"
```

### Terminal 2 — Scenario Filter (on host)

```bash
cd ~/ForSyDe-SystemC/examples/sadf/dstarlite_3d/gen
python3 scenario_filter.py top.xml self_report top_scenario.xml
```

This process blocks and waits. It updates `top_scenario.dot` in real time
each time the SADF scenario changes (NORMAL → REPLAN → NORMAL).

### Terminal 3 — ForSyDe-SystemC Simulation

```bash
docker exec ros_noetic bash -c \
  "source /opt/ros/noetic/setup.bash && \
   export LD_LIBRARY_PATH=/deps/systemc/lib-linux64:/opt/ros/noetic/lib && \
   cd /forsyde/examples/sadf/dstarlite_3d && \
   ./main.x"
```

---

## Expected Output (Terminal 3)

```
[Controller] Obstacle detected at grid (2,1)
[AbstractSys] Replanned from (1,1)
[Controller] Obstacle detected at grid (1,3)
[AbstractSys] Replanned from (1,2)
[Controller] Obstacle detected at grid (3,2)
[AbstractSys] Replanned from (2,2)
[Controller] Goal reached! Simulation complete.
```

The robot starts at grid cell (0,0), navigates to (5,5), and replans
3 times upon discovering unknown obstacles.

---

---

## Alternative Robot: Omni Robot (University Robot)

Instead of TurtleBot3, the same simulation can run on the omni-directional
robot developed at Shahid Beheshti University.

### Extra Prerequisites

Checkout the `omni-robot` branch of the worlds repository (the omni robot
model is bundled inside it — no extra clone needed):

```bash
cd ~/catkin_ws/src/dstarlite_maze
git checkout omni-robot
```

### Run (replace Terminal 1 command only)

**Terminal 1 — Gazebo with omni robot:**

```bash
xhost +local: && \
docker exec \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  ros_noetic bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source /catkin_ws/devel/setup.bash && \
   roslaunch dstarlite_maze dstarlite_omni.launch"
```

Terminals 2 and 3 are **identical** to the TurtleBot3 setup — no changes needed.

> The omni robot URDF is extended with a virtual LiDAR (`/scan`) and a
> planar-move plugin (`/cmd_vel` + `/odom`), so the ForSyDe simulation
> connects to it without any code changes.

---

## Project Structure

```
examples/sadf/dstarlite_3d/
├── main.cpp          # SystemC sc_main entry point
├── top.hpp           # Top-level module: wires all ForSyDe processes
├── globals.hpp       # Types, constants, grid helpers
├── monitor.hpp       # SDF combMN: LiDAR + odometry → monitor_state
├── saf.hpp           # SADF detectorMN: scenario transitions
├── controller.hpp    # SADF kernelMN A^L: D*-Lite + robot motion FSM
├── abstract_sys.hpp  # SADF kernelMN Ā^L: full D*-Lite replan
├── maze.hpp          # Grid allocation and D*-Lite initialization
├── heap.hpp          # Binary heap for D*-Lite priority queue
├── Makefile
└── gen/
    ├── top.xml             # ForSyDe introspection graph (auto-generated)
    ├── top_scenario.xml    # Filtered graph per active scenario
    ├── top_scenario.dot    # DOT visualization (updated at runtime)
    └── scenario_filter.py  # Reads self_report pipe, updates DOT graph
```
