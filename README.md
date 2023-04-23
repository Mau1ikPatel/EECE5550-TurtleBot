# EECE5550-TurtleBot
## Overview
This project is a solution to the disaster response prompt for EECE 5550: Mobile Robotics at Northeastern University. In this project, a TurtleBot robot was used to explore and map an unknown closed environment. Equipped with LIDAR and a camera, the robot collected data as it moved through the environment, using ROS to perform SLAM and generate a real-time map in the form of an occupancy grid. Additionally, a real-time AprilTag detector was developed to detect and track visual markers in the environment. The combination of the AprilTag detector and the SLAM mapping system would enable the robot to navigate the environment accurately and avoid obstacles in real-time. The project demonstrated the effectiveness of the TurtleBot and ROS in exploring and mapping an unknown environment and our implementation of the AprilTag detection shows the potential for computer vision applications such as object recognition and localization in disaster response. 

The contributors to this project included Maulik Patel, Jarrod Homer, Chris Swagler, Connor Nelson, and Aditi Purandare.

A video demonstration of this project can be found at https://youtu.be/6N2YskyDy9I.
### ROS Master
The ROS master used in development was an Ubuntu virtual machine (Virtualbox) on a Windows machine. ROS Noetic was installed in the VM and used as the ROS master.

### TurtleBot
The TurtleBot model used was a TurtleBot3 burger with a ROS Noetic Ubuntu Server image downloaded from the Robotis webpage. The hardware running this OS was a Raspberry Pi 3B+. On the configuration for the Pi, the ROS_MASTER_URI was set to the ROS master's IP address when connected on the same network. 

## Project Structure
For simplicity of the code submission, both the packages running on the ROS master and on the TurtleBot are included in this repository. The `turtlebot_packages` folder contains the packages used on the TurtleBot, and the other folders are the packages run on the ROS master.

Some of the old pose estimation is in the pose_estimation folder. This was not used as a node.
## Running the Project
### ROS Master
Start ROS with
```
roscore
```

### TurtleBot
ssh into the TurtleBot and run the following command to bring up basic packages to start the TurtleBot3
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Start the camera's raspicam_node with 
```
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true
```

Start the AprilTag continuous detection node with
```
roslaunch apriltag_ros continuous_detection.launch
```

### ROS Master

Run a static transformer to create a tf link between the base_link frame and the camera
```
rosrun tf static_transform_publisher 0.03 0 0.1 1.57095 0 1.57095 base_link camera_link 100
```

Run the tag_placer scripts to translate the AprilTag pose estimate into the map frame
```
rosrun tag_placer tagplacer.py
```

```
rosrun tag_placer tagtf.py
```


Run the SLAM node 
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Turn on move controls for autonomous movement
```
roslaunch turtlebot3_navigation move_base.launch
```

Run the frontier exploration node
```
roslaunch frontier_exploration explore_costmap.launch
```

Upon completion of environment exploration, save the map using
```
rosrun map_server map_saver -f ~/labcustom
```

## Missing Dependencies
Throughout the development of this project, the team frequently ran into issues with package dependencies going missing. Should there be issues running the code from this repository, install any missing packages with 
```
sudo apt-get install ros-noetic-<missing package>
```
