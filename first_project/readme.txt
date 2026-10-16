First Robotics Project - README
================================

HOW TO BUILD
------------
Place both packages in your ROS 2 workspace src/ directory:

  cp -r first_project ~/ros2_ws/src/
  cp -r bunker_msgs   ~/ros2_ws/src/

Build:

  cd ~/ros2_ws
  colcon build --packages-select bunker_msgs first_project
  source install/setup.bash

HOW TO RUN
----------
Terminal 1 - play a bag with simulated clock:

  ros2 bag play rosbag2_2026_04_08-16_38_55_fixed --clock

Terminal 2 - launch the project:

  ros2 launch first_project first_project.launch.py

HOW TO RESET ODOMETRY
---------------------
  ros2 service call /reset std_srvs/srv/Empty

NOTES
-----
- Use the fixed bags (bags_fixed/) for correct timestamps.
- The odometer reads linear_velocity and angular_velocity directly from
  BunkerStatus (already computed by the Agile-X driver from motor RPM).
- Odometry starts at (0, 0, 0). The GPS odometry (base_link) may start
  at a non-zero position - this is expected; the tf_error node computes
  the drift between the two paths.
- RViz shows: Red arrows = GPS (base_link), Blue arrows = computed (base_link2).
  Keep=100000 shows the full accumulated path.
