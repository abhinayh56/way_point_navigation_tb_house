# way_point_navigation_tb_house
This repository contains mapping, navigation, waypint input using map image and saving the coordinates and following the save coordinate to reach from initial to final goal and stop at destimation.

### Terminal 1 (runs the gazebo environment)
roslaunch turtlebot3_gazebo turtlebot3_house.launch

### Terminal 2 (start mapping using grid map)
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

### Terminal 3 (keyboard teleoperation)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

### Terminal 4 (save map to server)
rosrun map_server map_saver -f ~/map

### Terminal 5 (autonomous navigation using rviz)
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

### Terminal 6 (GUI for saving waypoints in txt file)
python save_wp.py

### Terminal 7 left (node for following saved waypoints)
rosrun tb3_pkg follow_wp.py

### Terminal 7 right (just to print velocity and position information)
rostopic echo /odom

### Terminal 8 (clustering using lidar for finding free spaces)
rosrun tb3_pkg lidar_clustering_2d.py
