# Project Go Chase It

In this project, there are two ROS packages in the catkin_ws/src: 

1. my_robot:

The my_robot ROS package has a robot, a white ball, and the world. The robot is a differential drive robot designed with the Unified Robot Description Format with two sensors:

* lidar hokuyo
* camera

To steer the robot a Gazebo plugin is added for robot’s differential drive, lidar, and camera.

The robot is housed inside the world I built in the Build My World project.

A white-colored ball is added to the Gazebo world which can be moved around and the robot will chase the ball when in its view.

To launch the packages, first the world.launch file should be launch which launches the world with the white-colored ball the robot.

2. ball_chaser:

The ball_chaser ROS package holds two C++ nodes. A drive_bot C++ node that provides a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. The service publishes to the wheel joints and returns back the requested velocities. The process_image C++ node reads the robot’s camera image and analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, the node request a service via a client to drive the robot towards it. The ball_chaser.launch should run both the drive_bot and the process_image nodes. The designed robot in this project will be used as a base model for all upcoming projects in this Robotics Software Engineer Nanodegree Program.

## File Structure
```
.Project2                          # Go Chase It Project
    ├── my_robot                       # my_robot package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── rc_robot.gazebo
    │   │   ├── rc_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── <yourworld>.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    └──
```
## Plugins

In the Udacity lesson a differential drive robot with two wheels was created. To steer the robot I used the skid steering drive plugin.
