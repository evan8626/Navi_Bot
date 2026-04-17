# Testing Navigation Bot
The goal of this project is to: 
<ol>
    <li> Continue building skills I have fostered in my professional career </li>
    <li> Make these skills a little more publically visible. </li>
</ol>
This project will not be ground breaking or feature any "WOW" tech. This is strictly meant to take what I know, build with it, and learn a bit along the way. This nav bot will employ the A* path planning algorithm for path planning. To follow that path, it will use the pure pursuit algorithm. Finally, for dyanmic response to changes in the path (an obstacle moves in front of the robot) DWA will be used.

Motion Control/Realtime Hardware
---
- STM32 SparkFun Thing Plus (https://www.sparkfun.com/sparkfun-thing-plus-stm32.html)
- Ultra sonic sensors (4 Fermion URM37 https://www.dfrobot.com/product-53.html)
- 2 2208 3-Phase Brushless DC Motors (12V 1800RPM, 300g.cm) (https://www.dfrobot.com/product-3007.html)
- SimpleFOCmini Brushless DC Motor Driver Board (https://www.dfrobot.com/product-3013.html)

The "real time" hardware will all be connected to the STM32 MC. The purpose of using the ultrasonic sensors on the real time hardware platform is to add a level of perception in case the more sophisticated higher level perception fails on the raspberry pi. Graceful handling of system failure is what is hoped to be achieved here.

Path Planning Hardware
---
- D-Robotics RDK X5 AI Developer Kit for ROS Robotics - 4GB, 10Tops (https://www.dfrobot.com/product-2945.html)
- RPLiDAR C1 - DTOF LiDAR 360° Laser Range Scanner (12m, IP54) (https://www.dfrobot.com/product-2803.html)

Power
---
- Drone Power Distribution Board XT60 3-4S 9-18V 5V 12V Output PDB (https://speedyfpv.com/products/drone-power-distribution-board-xt60-3-4s-9-18v-5v-12v-output-pdb?variant=8596736049203)
- 2 30A RC Brushless Motor Electric Speed Controller ESC 3A UBEC with XT60 & 3.5mm Bullet Plugs (https://www.amazon.com/RC-Brushless-Electric-Controller-bullet/dp/B071GRSFBD)
- Zeee 2S Lipo Battery 5200mAh 7.4V 80C Hard Case Deans T Plug for 1/8 1/10 RC Car (https://zeeebattery.com/products/zeee-2s-lipo-battery-7-4v-80c-5200mah-deans-plug-with-housing)

Languages
---
For the motion control C++ will be used and for the perception and planning Python 3.14 will be used

SDK
---
ROS2 - this will be on board the D-Robotics RDK developer board

Connection
---
The STM32 and the D-Robotics board will be connected and communicate over UART

Telemetry
---
A small telemetry server will be set up on a Windows based computer so a user can log in and see what the robot has mapped, and where the robot thinks it is in the map. This will be written primarily in CSS, html, and javascript as this will be a web-based telemetry app.

Project Structure
---
confg
    |
    ---- control_params.yaml   - lists control parameters for the robot for different control schemes.
    |
    ---- mission_config.yaml   - creates a small mission plan for the robot to pick up and drop off.
    |
    ---- robot_params.yaml     - lists size, speed, and sensor parameters of the robot.
launch
    |
    ---- robot.launch.py       - is the main launcher for the robot controller, the state machine, the path planner, and the simulated sensor nodes
    |
    ---- simulation.launch.py  - launcher for the simulator environment.
maps
    |
    ---- warehouse_simple.yaml - a simple 10 meter by 10 meter occupancy grid for the robot to navigate.
navi_bot
    |
    control
        |
        ---- kinematics.py - sets forward and reverse kinematic control for robot.
        |
        ---- motion_controller.py - PID controller for trajectory tracking and waypoint navigation.
        |
        ---- pure_pursuit.py - implements pure pursuit algorithm to determine how fast to follow path.
    |
    sensors
        |
        ---- imu_processor.py - initializes IMU and reads IMU sensor data.
        |
        ---- lidar_processing.py - initializes lidar and reads lidar data.
        |
        ---- odometry.py - initializes an odometry sensor and reads data.
    |
    utils
        |
        ---- geometry.py - common geometry calculations for distance and transforms
        |
        ---- profiler.py - monitors execution time, deadline misses, and resource usage
    |
    ---- __init__.py - sets version.
    |
    ---- mock_ros2.py - mock implementation of ROS2 nodes and message types without needing to install ROS2.
    |
    ---- path_planner.py - A*, D* Lite, and DWA planning algorithms are set up and run here.
    |
    ---- robot_controller.py - orchestrates all subsystems and coordinates multi-sensor readings.
    |
    ---- state_machine.py - robot's state machine.
tests
    |
    ---- AStar_plannner_test.py - tests A* implementation.
    |
    ---- DStar_Lite_planner_test.py - tests D* Lite implemetation.
    |
    ---- DWA_planner_test.py - tests DWA implementation.
    |
    ---- kinematics_test.py - tests kinematics implementation.
    |
    ---- path_planner_test.py - tests path planner implementation.
    |
    ---- pure_pursuit_test.py - tests pure pursuit implementation.
    |
    ---- state_machine_test.py - tests the state machine implementation.
    |
    ---- waypoint_nav_test.py - tests the robot's ability to navigate to waypoints.
package.xml - information about the project.
|
setup.py - sets up the project using information from package.xml.


