# Experimental Robotics Laboratory - Assignment 3
This project implements a simplified version of the game Cluedo, in which a robot moves around inside a square arena searching for hints to find the murderer, the weapon, and the location of the crime.

The hints are placed above the arena walls placed in the center of the four walls at corresponding waypoints and at different heights (0.75 and 1.25) that change each time the simulation is launched. To acquire the clues, the robot will have to reach for the different wayponints and use its arm to pick them up.

When the robot, after collecting a sufficient number of clues, finds a complete and consistent hypothesis, it reaches the central location and tells its hypothesis. If the hypothesis is the winning one, the game ends otherwise the robot resumes the search.

ARENA - from gazebo 

![ARENA](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/arena.png)

## Expected Behaviour
The robot should:

## Features of the project
The assignment requires:

# How it works

# Project structure

## Nodes
### Scripts folder
[hint_armor_ass3.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/hint_armor_ass3.py)

[robot_fsm.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_fsm.py)

[robot_vision.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_vision.py)

### Src folder
[marker_publish_camera1.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera1.cpp)

[marker_publish_camera2.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera2.cpp)

[marker_publish_camera3.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera3.cpp)

[simulation.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/simulation.cpp)

*If you need you can find a more accurate description of the nodes used in the documentation* [docs]

## Messages
[NewHint](https://github.com/piquet8/exp_ass3/blob/main/msg/NewHint.msg)

[NewHyp](https://github.com/piquet8/exp_ass3/blob/main/msg/NewHyp.msg)
## Services
[Marker.srv](https://github.com/piquet8/exp_ass3/blob/main/srv/Marker.srv)

## Urdf model
[robot.urdf](https://github.com/piquet8/exp_ass3/blob/main/urdf/robot9.urdf): this file contains the model of the robot

## UML
![UML_exp3](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/UML_exp3.png)

## States diagram
![states_diagram](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/blocks_diagram.png)

## Rqt-graph
![rqt_graph](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/rqt_graph.png)

# How to run the program
## Requirements
For run this project you will need the following packages:

## How to launch

1. Firstly, open the terminal, go to your workspace and in the src folder run:
```
git clone https://github.com/piquet8/exp_ass3.git
```
After that you need to build the package in the workspace: *catkin_make --only-pkg-with-deps exp_ass3*

# Video and images of the running program
- [VIDEO_DEMO](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/demo_exp3.mp4)

# Working hypothesis and environment

## System's features

## System's limitations

## System's technical improvements

# Specific link to folders  
- Here you can find the documentation: [docs]
- Here you can find the media and diagram file: [media and diagram](https://github.com/piquet8/exp_ass3/tree/main/media_exp3)

# Authors and contacts
AUTHOR: Gianluca Piquet

CONTACT: gianlucapiquet8@gmail.com 
