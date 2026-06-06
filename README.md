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
- **Telemetry** via a web-based dashboard showing live robot pose and mapped environment

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
| Telemetry dashboard | HTML / CSS / JavaScript | Web (Windows host) |

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
    index.html              # Dashboard UI (live telemetry + test runner tabs)
    main.js                 # Data rendering and live stream handling
    preload.js              # Button and navigation backend

tests/
    AStar_planner_test.py
    DStar_Lite_planner_test.py
    DWA_planner_test.py
    kinematics_test.py
    lidar_mock_test.py
    path_planner_test.py
    pure_pursuit_test.py
    state_machine_test.py
    waypoint_nav_test.py    # End-to-end integration test across full navigation stack
```

## Running the Tests

Each test module is a standalone script runnable without a ROS2 installation:

```bash
python tests/AStar_planner_test.py
python tests/waypoint_nav_test.py
# etc.
```

The mock ROS2 layer (`mock_ros2.py`) provides a lightweight message bus so components communicate through publishers and subscribers exactly as they would on real hardware.

## Status

| Component | Status |
|---|---|
| A* planner | Complete, tested |
| D* Lite planner | Complete, tested |
| DWA planner | Complete, tested |
| Pure Pursuit | Complete, tested |
| Kinematics | Complete, tested |
| State machine | Complete, tested |
| Path planner node | Complete, tested |
| Wheel odometry | Complete, tested |
| LIDAR processor | Complete, tested |
| Waypoint nav integration | Complete, tested |
| STM32 firmware (SimpleFOC) | Not started |
| UART protocol | Designed, not implemented |
| IMU processor | Stub |
| Telemetry dashboard | Stub |