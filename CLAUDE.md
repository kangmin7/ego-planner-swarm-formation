# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS 2 (Humble) port of EGO-Planner-Swarm — a quadrotor swarm trajectory planner (B-spline based gradient optimization + kinodynamic A*) plus a Gazebo/RViz simulation stack (fake drone dynamics, depth-camera local sensing, map generation). This repo lives at `src/ego-planner-swarm` inside a larger colcon workspace (`~/ros2_ws`) that contains many unrelated packages (aerostack2, navigation2, px4_*, etc.) — always scope builds with `--packages-select`/`--packages-up-to` to avoid rebuilding the whole workspace.

## Build

From the workspace root (`~/ros2_ws`), not from this repo directory:

```bash
cd ~/ros2_ws
colcon build --packages-up-to ego_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

To rebuild a single package after editing it: `colcon build --packages-select <pkg_name>`.

Packages build with `-O3 -Wall -Wextra -Wpedantic`, C++17, `CMAKE_BUILD_TYPE` forced to `Release` in each `CMakeLists.txt`.

### Required system deps
- PCL, and VTK (PCL's dependency — must enable Qt when building VTK)
- `ros-humble-rmw-cyclonedds-cpp` — FastDDS (ROS 2 default) causes severe lag with this stack's pub/sub pattern; switch the RMW implementation:
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && source ~/.bashrc
  ros2 doctor --report | grep "RMW middleware"   # must print rmw_cyclonedds_cpp
  ```

## Running

```bash
ros2 launch ego_planner rviz.launch.py            # visualization, separate terminal
ros2 launch ego_planner single_run_in_sim.launch.py   # single drone
ros2 launch ego_planner swarm.launch.py               # swarm
ros2 launch ego_planner swarm_large.launch.py          # large swarm
```

Optional launch args: `use_mockamap:=True` (mockamap instead of default Random Forest map generator), `use_dynamic:=True` (enable dynamic-obstacle handling). Example:
```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

There is no test suite beyond the default `ament_lint_auto`/`ament_lint_common` hooks wired into each `CMakeLists.txt` under `BUILD_TESTING`.

## Architecture

Two top-level package groups under `src/`:

### `src/planner/` — the planning stack
- **`plan_manage` (ROS package name `ego_planner`)** — top-level node and the replanning state machine.
  - `ego_planner_node.cpp` — node entry point.
  - `ego_replan_fsm.{h,cpp}` (`EGOReplanFSM`) — the core replanning loop, a state machine with states `INIT → WAIT_TARGET → GEN_NEW_TRAJ/REPLAN_TRAJ → EXEC_TRAJ → EMERGENCY_STOP`, plus `SEQUENTIAL_START` for swarm staggered launch. Owns odometry/target callbacks, decides when to trigger replanning vs. emergency stop, and publishes `traj_utils/msg/Bspline` (and `MultiBsplines` for swarm coordination).
  - `planner_manager.{h,cpp}` (`EGOPlannerManager`) — glue between the FSM and the optimization/search backends; drives the actual trajectory generation pipeline (initial path search → B-spline fit → optimization → safety checks).
  - `traj_server.cpp` — separate node that samples the published B-spline trajectory at control rate and feeds the controller/simulator.
- **`path_searching`** — `dyn_a_star` (kinodynamic-aware A*) for the initial coarse path search.
- **`bspline_opt`** — `UniformBspline` (B-spline representation/evaluation) and `BsplineOptimizer` (gradient-based trajectory refinement using an internal `lbfgs.hpp` L-BFGS solver and `gradient_descent_optimizer`), used by `planner_manager` to turn the coarse path into a smooth, dynamically-feasible, collision-free trajectory.
- **`plan_env`** — environment representation consumed by both search and optimization: `grid_map` (occupancy/ESDF-style map from depth sensing), `raycast`, `obj_predictor`/`linear_obj_model.hpp` (predicting dynamic obstacle motion), `obj_generator` (synthetic moving obstacles for sim).
- **`traj_utils`** — shared message definitions (`Bspline.msg`, `MultiBsplines.msg`, `DataDisp.msg`) used for inter-node and inter-drone trajectory communication, plus `planning_visualization` for RViz markers.
- **`drone_detect`** — inter-drone detection/avoidance support for swarm scenarios.
- **`rosmsg_tcp_bridge`** — bridges planning messages over TCP, used for swarm communication across separate ROS domains/machines (e.g. real multi-drone deployments).

Data flow: `plan_env` (map) + `path_searching` (coarse path) feed into `bspline_opt` (smoothing/optimization), orchestrated by `planner_manager`, driven by the `ego_replan_fsm` state machine, with results sampled by `traj_server` and sent to the controller.

### `src/uav_simulator/` — simulation/visualization stack
- **`so3_quadrotor_simulator`** + **`so3_control`** — quadrotor dynamics simulation and SO(3) geometric controller that consumes sampled trajectories from `traj_server` and produces simulated odometry.
- **`fake_drone`** — lightweight stand-in drone model for quick testing without full dynamics.
- **`local_sensing`** — simulates depth-camera/lidar perception, feeding `plan_env`'s `grid_map`.
- **`map_generator`** / **`mockamap`** — synthetic environment generation (Random Forest by default, or mockamap when `use_mockamap:=True`).
- **`Utils/`** — shared utility packages: `quadrotor_msgs` (message defs shared across sim and planner), `pose_utils`, `uav_utils`, `odom_visualization`, `multi_map_server`, `waypoint_generator`, `cmake_utils`.

### Known issue
ROS 2 default RMW (FastDDS) causes significant latency in this stack's pub/sub pattern (root cause not identified) — always use `cyclonedds` per the Build section above when running or debugging timing-sensitive behavior.
