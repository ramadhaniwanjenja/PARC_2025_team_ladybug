TEAM 1: PARC Engineers League
Introduction
The PARC 2025 Autonomy Track challenges teams to develop autonomous navigation solutions for agricultural robots operating in crop environments. Precision agriculture represents a transformative approach to farming that uses technology to optimize crop yields while minimizing resource waste through precise application of water, fertilizers, and pesticides. While adoption offers significant benefits including increased efficiency, reduced environmental impact, and improved crop monitoring capabilities, farmers face barriers such as high initial investment costs, technical complexity, and the need for specialized training. Our solution addresses the critical need for reliable autonomous navigation in structured agricultural environments, enabling robots to traverse crop rows efficiently while avoiding plant damage.
Team Country: Rwanda
Team Member Names:

Cyusa Impano Chrispin (team leader)
Tesi Eva Gasana
Dufitimana Reponse
Simugomwa Noella Divine 
Ishimwe Karamage Yves
Godwin De Godson

Dependencies
Packages needed are:

rclpy: Python client library for ROS 2

$ sudo apt-get install ros-jazzy-rclpy


geometry_msgs: ROS 2 geometry message definitions for Twist commands

$ sudo apt-get install ros-jazzy-geometry-msgs


nav_msgs: Navigation message definitions for odometry data

$ sudo apt-get install ros-jazzy-nav-msgs


sensor_msgs: Sensor message definitions for LaserScan data

$ sudo apt-get install ros-jazzy-sensor-msgs


numpy: Numerical computing library for efficient array operations

$ pip3 install numpy



Task
Our approach implements a speed-optimized navigation system using pre-calibrated distance and angle parameters for each world environment. The solution employs direct odometry-based positioning with LIDAR obstacle detection, utilizing increased movement speeds (2.5 m/s linear, 1.0 rad/s angular) and multi-threaded execution for enhanced performance. The robot follows precise waypoint navigation through crop rows using world-specific parameter sets that have been calibrated for optimal path traversal. Our system features adaptive speed control during obstacle detection, heading correction for straight-line movement, and efficient computational processing with reduced update cycles. The solution prioritizes completion time while maintaining plant avoidance through strategic front-arc obstacle detection and emergency stop capabilities.
Command to run solution:
