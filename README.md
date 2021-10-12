# Hand Gesture Controlled Robot
This project aim is to controll turtlebot3 robot in gazebo using hand gestures.
## Requirements
1. Install Ros From link:
http://wiki.ros.org/ROS/Installation (Noetic-Version)
2. Create a catkin_ws in your home directory:
```
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/tutorial_ws/src
$ cd ~/tutorial_ws
$ catkin_init_workspace src
$ catkin_make
```
3. Install these packages in catkin_ws
``` 
$ sudo apt-get install ros-noetic-joint-state-publisher
$ sudo apt-get install ros-noetic-joint-state-publisher-gui
```
4. Install Ros control in catkin_ws from link:
http://wiki.ros.org/ros_control
5. Install Turtlebot3 Packages in src folder of catkin_ws :
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
## Main concepts 
1. Primary concepts used for hand gesture recognition are contour,convex-hull and convexity-defects.
2. For controlling the Turtlebot3 we published velocity to turtlebot3 in gazebo using rostopic /cmd_vel
## Getting Started
1. Start your Turtlebot3 simulation:
```
$ cd ~/catkin_ws 
$ export TURTLEBOT3_MODEL=waffle
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
2. Clone the code and run the code from any code editor or terminal.
## Demonstration
### Hand Gesture Recognition

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/85958512/136939977-4b3db8a9-410b-404a-9630-0e4c475ec77c.gif)

### Hand Gesture Controlled Robot

![ezgif com-gif-maker (6)](https://user-images.githubusercontent.com/85958512/136952213-14df85a5-0b95-42b9-a244-de09fec2fdd4.gif)
