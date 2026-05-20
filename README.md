# ROS2 port of franka_ros for Franka Emika Panda (FER) robots

This project is for porting over the various functionalities from franka_ros into ROS2 for Panda robots.
Franka Emika has dropped software support for robots older than FR3, which leaves a lot of older hardware outdated and unable to migrate to ROS2.

This repository attempts to remedy that somewhat, by bringing existing features from franka_ros over to ROS2 specifically for the Panda robots.

<!-- ## Credits -->
<!-- The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble]. -->

<!-- ## Installation Guide -->

<!-- (Tested on Ubuntu 24.04, ROS2 Humble, Panda FCI 4.0.4, 4.2.2 and 4.2.1, and Libfranka 0.8.0 and 0.9.2) -->

<!-- 1. Build libfranka 0.8.0 or 0.9.2 from source by following the [instructions][libfranka-instructions]. Choose proper version according to your FCI version! If you need to install libfranka 0.9.2, you can use directly LCAS [libfranka 0.9.2][libfranka-LCAS] release -->
<!-- 2. Install FLIR Blackfly_s camera ROS2 driver (required for panda_vision setup), following the [instructions][flir_camera_driver] -->
<!-- 3. Clone this repository into your workspace's `src` folder. -->
<!-- 4. Source the workspace, then in your workspace root, call: --> 
<!-- ```bash -->
<!-- colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build` -->
<!-- ``` -->
<!-- 5. Add the build path to your `LD_LIBRARY_PATH`: `LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"` -->
<!-- 6. To test, source the workspace, and run: --> 
<!-- ```bash -->
<!-- ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py robot_ip:=<fci-ip> camera_type:=blackfly_s serial:="'<camera-serial>'" load_camera:=True planner:=<planner_name> -->
<!-- ``` -->
<!-- Example `robot_ip:=172.16.0.2`, `serial:="'22141921'"`, `planner:=pilz_industrial_motion_planner/CommandPlanner` --> 

<!-- If needed to test on fake hardware add `use_fake_hardware:=True` argument to the launch file -->

<!-- 7. To control the arm by MoveIt2 for plant scanning, please follow [moveit2_commander_recorder][moveit2_commander_recorder] and [viewpoint_generator][viewpoint_generator] repositories. -->
