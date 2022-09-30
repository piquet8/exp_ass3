# Experimental Robotics Laboratory - Assignment 3
This project implements a simplified version of the game Cluedo, in which a robot moves within a complex environment consisting of several rooms searching for hints to find the murderer, the weapon, and the location of the crime.

The hints are inside 30 ArUco markers (5 for each room) that can be placed in different locations within the rooms: above the wall, at the base of the wall, or on the floor. To acquire the hints, the robot will have to move around the different rooms and use its camera to detect the different markers.

When the robot, after collecting a sufficient number of hints, finds a complete and consistent hypothesis, it reaches the home position and communicates its hypothesis. If the hypothesis is the winning one, the game ends, otherwise the robot resumes the search.

ARENA - from gazebo 

![ARENA](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/arena.png)

## Expected Behaviour
The robot should:
- moving between different rooms while obviously avoiding hitting the walls
- detect the different ArUco markers placed at different locations within the rooms to obtain hints
- when a consistent hypothesis is deducible, it should go the center of the arena and express it in English
- if the hypothesis is wrong, it should keep exploring and find new hints

## Features of the project
The assignment requires:
- use **ArUco** to detect markers that contain the hints
- use functionalities such as **mapping, path planning and following** to allow the robot to move between rooms
- modify the robot model with **multiple sensors, movable joints, etc.** to enable it to detect markers


# How it works

# Project structure

## Nodes
### Scripts folder
[hint_armor_ass3.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/hint_armor_ass3.py): this node implements the cluedo ontology of the robot, it takes hints from the topic */new_hint* and manages them to achieve a complete and consistent hypothesis. It's the same node of the previous assignment, you can find more information there [exp_ass1](https://github.com/piquet8/exp_ass1)

[robot_fsm.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_fsm.py): this node implements the finite state machine that determines the behaviors of the robot during program execution. In particular, we have 3 states: 
- **state 0** *goto_room*: in this state one of the 6 rooms in the environment is randomly chosen and this is set as the goal for the move_base client, every time a room is seen it is added to the list of visited rooms, in this way through a check in the list we are sure to check all rooms even if in random order, once all rooms have been seen the list is emptied. After this state we usually go to state 1, however to be on the safe side it is first checked if on the way no hypothesis was found that needs to be tested, in that case we go directly to state 2.

- **state 1** *look_aroud*: In this state the robot is in one of the rooms and here it must find the clues. In order for the robot to be able to find as many markers as possible, different behaviors have been implemented depending on the size of the rooms and the placement of the markers. For example in smaller rooms the robot rotates on itself 360 degrees using the turn_around() function, in larger rooms on the other hand in addition to performing this rotation it moves to different points in the room. Again this is checked to see if the robot has found a complete hypothesis to test in order to move to state 2, if not, the robot will have to continue searching in the other rooms and then return to state 0

- **state 2** *go_home*: in this state the robot has found a complete hypothesis to test and heads to the home position located in the center of the arena. The check_win() function is used to check if the id of the hypothesis found matches the winning id. If the hypothesis is correct the game ends otherwise the robot resumes the search and then goes to state 0 again

[robot_vision.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_vision.py)

### Src folder
[marker_publish_camera1.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera1.cpp)

[marker_publish_camera2.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera2.cpp)

[marker_publish_camera3.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera3.cpp)

[simulation.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/simulation.cpp)

*If you need you can find a more accurate description of the nodes used in the documentation* [docs](https://github.com/piquet8/exp_ass3/tree/main/docs)

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
- Here you can find the documentation: [docs](https://github.com/piquet8/exp_ass3/tree/main/docs)
- Here you can find the media and diagram file: [media and diagram](https://github.com/piquet8/exp_ass3/tree/main/media_exp3)

# Authors and contacts
AUTHOR: Gianluca Piquet

CONTACT: gianlucapiquet8@gmail.com 
