# Navi Bot

A ROS2-based autonomous differential drive robot implementing classical navigation algorithms on physical hardware. The project spans embedded motor control on an STM32 microcontroller through high-level path planning and mission execution on a D-Robotics RDK X5.

## Goals

- Apply and extend skills in embedded systems, robotics, and software architecture in a hands-on hardware project
- Build a well-structured, tested codebase that demonstrates autonomous navigation from the ground up

## What it does

Navi Bot autonomously navigates a mapped environment, accepts pickup and delivery missions, and replans dynamically when obstacles appear. The software stack covers:

- **Global path planning** via A* on an occupancy grid
- **Dynamic replanning** via D* Lite when the environment changes
- **Local trajectory following** via Pure Pursuit and DWA
- **Mission management** via a state machine (IDLE → PICK_NAV → PICKING_UP → DELIVERY_NAV → DELIVERING → IDLE)
- **Pose estimation** via wheel odometry with encoder feedback
- **Obstacle detection** via LIDAR with DBSCAN clustering and costmap inflation
- **Telemetry & tooling** via a desktop dashboard (Electron) with three tabs: a live WebSocket feed of robot pose and map, an interactive planner **Demo** (click to place start/goal, watch A*/D* Lite/DWA solve in real time), and a built-in **Test Runner** that streams results case-by-case and animates each planner driving across its actual occupancy grid

## Hardware

### Motion Control (Real-time layer)
| Component | Part |
|---|---|
| Microcontroller | SparkFun Thing Plus STM32 |
| Ultrasonic sensors (×4) | DFRobot Fermion URM37 |
| Drive motors (×2) | 2208 3-Phase BLDC, 12V 1800RPM |
| Motor drivers (×2) | SimpleFOCmini BLDC Driver |

The STM32 handles real-time motor control and ultrasonic-based fallback obstacle detection, providing a safety layer if higher-level perception fails.

### Planning and Perception (High-level layer)
| Component | Part |
|---|---|
| SBC / compute | D-Robotics RDK X5 AI Dev Kit (4GB, 10 TOPS) |
| LIDAR | RPLiDAR C1 DTOF 360° (12m, IP54) |

### Power
| Component | Part |
|---|---|
| Power distribution | XT60 3-4S PDB (5V / 12V output) |
| ESCs (×2) | 30A Brushless ESC with XT60 |
| Battery | Zeee 2S LiPo 5200mAh 7.4V 80C |

## Software Stack

| Layer | Language | Platform |
|---|---|---|
| Motion control / firmware | C++ | STM32 |
| Planning, perception, state management | Python 3 | RDK X5 / ROS2 |
| Telemetry dashboard | HTML / CSS / JavaScript (Electron) | Desktop (Windows host) |

Communication between the STM32 and RDK X5 is over UART using a fixed-length binary packet protocol.

## Project Structure

```
config/
    control_params.yaml     # Control parameters for different schemes
    mission_config.yaml     # Mission plan (pickup and delivery waypoints)
    robot_params.yaml       # Robot geometry, speed, and sensor parameters

launch/
    robot.launch.py         # Main launcher (controller, state machine, path planner, sensors)
    simulation.launch.py    # Simulation environment launcher

maps/
    warehouse_simple.yaml   # 10m × 10m occupancy grid

navi_bot/
    control/
        kinematics.py       # Forward and inverse differential drive kinematics
        motion_controller.py # PID controller for trajectory tracking
        pure_pursuit.py     # Pure Pursuit steering controller

    planners/
        astar.py            # A* global path planner
        dstar_lite.py       # D* Lite dynamic replanner
        dwa.py              # Dynamic Window Approach local planner

    sensors/
        imu_processor.py    # IMU initialization and data reading
        lidar_processing.py # LIDAR scan processing, obstacle clustering, costmap update
        odometry.py         # Wheel odometry pose estimation

    utils/
        geometry.py         # Distance, angle, and coordinate transform utilities
        profiler.py         # Execution time monitoring and deadline miss tracking
        websocket.py        # WebSocket server for telemetry frontend

    mock_ros2.py            # Lightweight ROS2 mock (nodes, publishers, subscribers, message bus)
    path_planner.py         # A*, D* Lite, and DWA integration node
    robot_controller.py     # Top-level subsystem orchestration
    state_machine.py        # Mission state machine

telemetry_dashboard/
    index.html              # Dashboard UI — Live feed, interactive Demo, and Test Runner tabs
    main.js                 # Electron main process — runs test subprocesses, native dialogs, IPC
    preload.js              # contextBridge IPC API (runTest, listTests, stopTest, pickDir)

tests/
    AStar_planner_test.py          # A* unit tests + differential optimality vs reference Dijkstra
    DStar_Lite_planner_test.py     # D* Lite unit + incremental-replan (k_m / edge-change) tests
    DWA_planner_test.py            # DWA closed-loop avoidance tests
    astar_dwa_integration_test.py  # A* plans a global path, DWA follows it (global + local)
    dstar_dwa_integration_test.py  # D* Lite plans + replans, DWA follows the updated path
    waypoint_nav_test.py           # End-to-end mission run across the full navigation stack
    kinematics_test.py             # Differential drive forward/inverse kinematics invariants
    motion_controller_test.py      # PID terms, anti-windup, and closed-loop convergence
    pure_pursuit_test.py           # Pure Pursuit steering/lookahead invariants
    odometry_test.py               # Wheel odometry dead-reckoning and velocity estimation
    lidar_mock_test.py             # Scan processing, clustering, and costmap inflation
    geometry_test.py               # Transform, distance, collision, and interpolation utilities
    profiler_test.py               # Timing, deadline tracking, and statistics
    path_planner_test.py           # Path planner node integration
    state_machine_test.py          # Mission state machine transitions
    imu_test.py                    # spec ahead of implementation (IMU module in progress)
    websocket_test.py              # spec ahead of implementation (telemetry server)
```

## Testing

Each test module is a standalone script runnable without a ROS2 installation. The mock ROS2 layer (`mock_ros2.py`) provides a lightweight message bus so components communicate through publishers and subscribers exactly as they would on real hardware.

```bash
python tests/AStar_planner_test.py          # run a whole suite
python tests/AStar_planner_test.py 7        # run a single test (1-indexed)
python tests/AStar_planner_test.py --list   # list a suite's test cases
```

The test approach mixes a few styles:

- **Unit / invariant tests** check each module against physical and mathematical invariants (e.g. kinematics round-trips, PID anti-windup).
- **Differential tests** validate planner output against an independent reference (A*/D* Lite paths are checked for cost-optimality and corner safety against a reference Dijkstra implemented inside the test).
- **Integration tests** exercise the full global + local stack: A* or D* Lite plans a route, DWA follows it with a lookahead controller, and the suite asserts the robot reaches the goal without collisions — including a moving-obstacle dodge and D* Lite incremental replanning when the map changes.

The whole suite also runs in the Electron dashboard's **Test Runner** tab, which streams output one case at a time, badges pass/fail per suite, draws each planner's path animating across its real occupancy grid, and lets you hover the grid to scrub the robot back and forth along a finished run. CI runs the active suites on every push via GitHub Actions.

### Running the dashboard

```bash
cd telemetry_dashboard
npm install
npm start
```

## Status

| Component | Status |
|---|---|
| A* planner | Complete, tested |
| D* Lite planner | Complete, tested (incremental replanning) |
| DWA planner | Complete, tested |
| Pure Pursuit | Complete, tested |
| Kinematics | Complete, tested |
| Motion controller (PID) | Complete, tested |
| State machine | Complete, tested |
| Path planner node | Complete, tested |
| Wheel odometry | Complete, tested |
| LIDAR processor | Complete, tested |
| Geometry / profiler utilities | Complete, tested |
| Global + local integration (A*/D* Lite → DWA) | Complete, tested |
| Waypoint nav integration | Complete, tested |
| Telemetry dashboard | Functional (live feed, demo, test runner) |
| IMU processor | In progress (test spec written) |
| WebSocket telemetry server | In progress (test spec written) |
| STM32 firmware (SimpleFOC) | Not started |
| UART protocol | Designed, not implemented |