# Experimental Robotics Laboratory - Assignment 3
This project implements a simplified version of the game Cluedo, in which a robot moves within a complex environment consisting of several rooms searching for hints to find the murderer, the weapon, and the location of the crime.

The hints are inside 30 ArUco markers (5 for each room) that can be placed in different locations within the rooms: above the wall, at the base of the wall, or on the floor. To acquire the hints, the robot will have to move around the different rooms and use its camera to detect the different markers.

When the robot, after collecting a sufficient number of hints, finds a complete and consistent hypothesis, it reaches the home position and communicates its hypothesis. If the hypothesis is the winning one, the game ends, otherwise the robot resumes the search.

MAZE - from gazebo 

![MAZE](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/arena.png)

## Expected Behaviour
The robot should:
- moving between different rooms while obviously avoiding hitting the walls
- detect the different ArUco markers placed at different locations within the rooms to obtain hints
- when a consistent hypothesis is deducible, it should go the center of the maze and express it in English
- if the hypothesis is wrong, it should keep exploring and find new hints

## Features of the project
The assignment requires:
- use **ArUco** to detect markers that contain the hints
- use functionalities such as **mapping, path planning and following** to allow the robot to move between rooms
- modify the robot model with **multiple sensors, movable joints, etc.** to enable it to detect markers


# How it works
In this project, the robot's goal is to move between different rooms and look for markers containing hints by which it can find the correct hypothesis. Thus, the main aspects of the problem are the implementation of a navigation system that allows the robot to move between rooms while avoiding walls and a marker detection system that allows the robot to collect clues. To satisfy the first aspect I used the **move_base** package in particular the use of its action that, given a target within the environment will attempt to reach it. This node connects a global and local planner to perform its global navigation task. In particular, this robot has a *laser_scan* that is used to provide information about what the robot encounters along the way by updating the local_costmap and global_costmap. To solve the second aspect instead, we use the *ArUco* module that is based on the **Aruco library** that provides the *aruco_detect* function that can detect ArUco markers. An ArUco marker is a synthetic square marker consisting of a large black border and an internal binary matrix that determines its identifier (you find an image below). To enable the robot to find as many clues as possible, the robot was provided with 3 cameras, placed on the base of the robot, on an inclined arm of the robot, and on the end-effector of the robot. The idea was to have a camera that can detect the markers in whatever position they are in fact the camera placed on the end-effector finds the markers located above the wall, the camera placed on the inclined arm looking downwards finds the markers on the floor while the camera on the base of the robot finds the markers located below the base of the wall (you find an image below). In addition low speeds were used so that the images obtained from the cameras were as stable and sharp as possible and different behaviors were implemented to search for hints inside the cameras. They depending on the room size, in a small room a single 360-degree turn of the robot on itself may be enough to acquire all the clues around it, in a larger room this may not be enough so more points were set to be reached by the robot in the same room in order to achieve a more complete search.

![mar](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/mark.png)      ![rob](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/robotpic.png) 

# Project structure
The project consists of 7 nodes that communicate with each other and with related modules. The two main nodes I have implemented are the [robot_fsm.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_fsm.py) node and the [robot_vision.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_vision.py) node: the former implements the finite state machine that controls the behavior of the robot throughout the simulation while the latter implements the vision part of the robot that enables the detection, collection and processing of hints. Three states are implemented in the robot_fsm.py node: the *goto_room* state that allows the robot with the use of the **move_base** module to move between rooms, the *look_around* state that allows the robot to search inside the rooms for hints and finally the *go_home* state that leads the robot to the home location and checks if the hypothesis found is the winning one. The robot_vision.py node, on the other hand, allows the robot to identify markers in the environment through the use of the cameras on the robot and extract from them the clues needed to find complete and consistent hypotheses to be tested. To identify the ArUco markers present in the environment this node subscribes to the 3 nodes [marker_publish_camera1.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera1.cpp), [marker_publish_camera2.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera2.cpp), [marker_publish_camera3.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera3.cpp) that used the **ArUco** module to acquire the ids related to the markers seen by the cameras. Whenever a hints is found after being processed it is stored by the [hint_armor_ass3.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/hint_armor_ass3.py) node, which is responsible for finding complete and consistent hypotheses to allow the robot to try to solve the crime by finding the right hypothesis. The last node to mention is the one provided by default in the assignment, the [simulation.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/simulation.cpp); this node is in charge of implementing the different elements in the environment and the generation of clues and solutions. 

*This is a brief description of what happens inside the system, in the following sections it will be possible to better analyze the nodes specifically and their connections*

## Nodes
### Scripts folder
[hint_armor_ass3.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/hint_armor_ass3.py): this node implements the cluedo ontology of the robot, it takes hints from the topic */new_hint* and manages them to achieve a complete and consistent hypothesis. It's the same node of the previous assignment, you can find more information there [exp_ass1](https://github.com/piquet8/exp_ass1)

[robot_fsm.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_fsm.py): this node implements the finite state machine that determines the behaviors of the robot during program execution. In particular, we have 3 states: 
- **state 0** *goto_room*: in this state one of the 6 rooms in the environment is randomly chosen and this is set as the goal for the `move_base client`, every time a room is seen it is added to the list of visited rooms, in this way through a check in the list we are sure to check all rooms even if in random order, once all rooms have been seen the list is emptied. After this state we usually go to state 1, however to be on the safe side it is first checked if on the way no hypothesis was found that needs to be tested, in that case we go directly to state 2.

- **state 1** *look_aroud*: In this state the robot is in one of the rooms and here it must find the clues. In order for the robot to be able to find as many markers as possible, different behaviors have been implemented depending on the size of the rooms and the placement of the markers. For example in smaller rooms the robot rotates on itself 360 degrees using the *turn_around()* function (this function publish on the topic `/cmd_vel` an angular velocity with respect to the z-axis allowing rotation in place of the robot), in larger rooms on the other hand in addition to performing this rotation it moves to different points in the room. Again this is checked to see if the robot has found a complete hypothesis to test in order to move to state 2, if not, the robot will have to continue searching in the other rooms and then return to state 0

- **state 2** *go_home*: in this state the robot has found a complete hypothesis, get it from the `/hypothesis` topic. To test it, the robot goes to the home position located in the center of the maze. The *check_win()* function is used to check if the id of the hypothesis found matches the winning id. If the hypothesis is correct the game ends otherwise the robot resumes the search and then goes to state 0 again

[robot_vision.py](https://github.com/piquet8/exp_ass3/blob/main/scripts/robot_vision.py): this node implements the robot vision; in fact, it uses the robot's cameras to detect markers and obtain hints. The node is very simple because it implements 3 equal functions each dedicated to a different camera, so it will be sufficient to explain the operation of one of them. The function cam_newId() subscribes to the topic `/camera_publish/camera_found_id` to get information from the camera present on the robot and in particular to get the value of the marker that is detected by the camera. Two checks are done, it is checked that the id displayed is within the range of possible values 11 - 40 (it may happen that the camera for different reasons reads a wrong value) and it is checked that the id found has not been displayed before. Once this id is obtained it is used as an argument for the call to the `oracle_hint` service that provides in response the hint corresponding to the marker found. It is checked whether the format of the hintis correct, and if so, the hint information is processed into a single string and publish to the `/new_hint` topic

### Src folder
The next three nodes, are a variant of this node [marker_publish.cpp](https://github.com/CarmineD8/aruco_ros/blob/main/aruco_ros/src/marker_publish.cpp); this node is used to acquire information from the cameras on the robot; I added a publisher that publishes on the topic *camera_found_id* the id found by the *detect* function provided by the **ArUco** module. This function takes as arguments the the image containing the markers to be detected and the  the parameters that can be customized during the detection process. In this way it is sufficient to pass as an argument in the [launch](https://github.com/piquet8/exp_ass3/blob/main/launch/simulation.launch) file `/image:=/robot/camera/image_raw` with the respective camera number (camera1, camera2, camera3) to the corresponding dedicated node to receive the id of the marker detected by the camera on the topic `/camera_publish/camera_found_id`

[marker_publish_camera1.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera1.cpp)

[marker_publish_camera2.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera2.cpp)

[marker_publish_camera3.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/marker_publish_camera3.cpp)

[simulation.cpp](https://github.com/piquet8/exp_ass3/blob/main/src/simulation.cpp): this node is implemented by default you can find it in the assignment package [exp_assignment3](https://github.com/CarmineD8/exp_assignment3/tree/main/src). This node implements all the simulation aspects, in particular it provides the values of the hints related to the different markers through the service `/oracle_hint` and the value of the id of the winning hypothesis with the service `oracle_solution` 

*If you need you can find a more accurate description about the nodes implemented my myself in the documentation* [docs](https://github.com/piquet8/exp_ass3/tree/main/docs)

## Messages
[NewHint.msg](https://github.com/piquet8/exp_ass3/blob/main/msg/NewHint.msg): this message contains the hint taken by the robot redesigned to be more readable in only one string, it is publish on the `/new_hint` topic

[NewHyp.msg](https://github.com/piquet8/exp_ass3/blob/main/msg/NewHyp.msg): this message contains the hypothesis complete, it is publish on the `/hypothesis` topic
## Services
[Marker.srv](https://github.com/piquet8/exp_ass3/blob/main/srv/Marker.srv): this service takes as request the value of the id corresponding to the detected marker and provides as response the corresponding hint which is taken from the message *ErlOracle.msg* present in the package [erl2](https://github.com/CarmineD8/erl2/tree/main/msg); to call the service we can use `oracle_hint`

## Urdf model
[robot.urdf](https://github.com/piquet8/exp_ass3/blob/main/urdf/robot9.urdf): this file contains the model of the robot

## UML
![UML_exp3](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/UML_exp3.png)

## States diagram
![states_diagram](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/blocks_diagram.png)

## Rqt-graph
![rqt_graph](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/rqt_graph.png)

## Extra package

Within the assignment package we find the package [exp3_move](https://github.com/piquet8/exp_ass3/tree/main/exp3_move) this package was used to allow the robot arm to move however I later chose to use a strategy of not moving the arm to acquire the hints because the movement of the arm not always linear produced an incorrect reading of the markers. This package still allows for the implementation of arm movement functions should the other programmers need to change strategies and move the arm during the simulation.

# How to run the program
## Requirements
For run this project you will need the following packages:

- **aruco_ros** that you can find here: https://github.com/CarmineD8/aruco_ros
- **armor** you can find the installation procedure here: https://github.com/EmaroLab/armor
- **move_base** that you can find here: https://github.com/ros-planning/navigation.git
- **erl2** thet you can find here: https://github.com/CarmineD8/erl2.git

*SUGGESTION: you should find these packages already installed in this docker workspace:* https://hub.docker.com/r/carms84/exproblab

## How to launch

1. Firstly, open the terminal, go to your workspace and in the src folder run:
```
git clone https://github.com/piquet8/exp_ass3.git
```
After that you need to build the package in the workspace: *catkin_make --only-pkg-with-deps exp_ass3*

2. Then to launch the simulation environment and relative nodes open a new shell tab and run the command:
```
roslaunch exp_ass3 simulation.launch
```
*SUGGESTION: For a cleaner terminal with no warning messages I suggest you to try to use this command to launch:*
```
roslaunch exp_ass3 simulation.launch 2>/dev/null
```
3. Finally to starts the game open a new shell tab and run the command:
```
rosrun exp_ass3 robot_fsm.py
```

# Video and images of the running program
[VIDEO_DEMO](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/demo_exp3.mp4)

In this video it is possible to view a demo of the robot in action: 
- on the left you will find the Gazebo simulation where the robot moves within the environment 
- on the top right instead you can see the Rviz interface divided into two parts, 
  - in the left part there are 3 panels that show the real-time images taken by the cameras mounted on the robot, 
  - in the right part you can see the robot within the map that is updated and outlined as the robot moves in the environment and a green line that represents the path planned by the robot to reach its goal
- on the botton right we find the terminal on which the messages related to the simulation scroll; again we have a division into two finistries
  - on the left of terminal we can see the messages produced by the robot_vision node that report: which camera has detected a marker, the value of its id and the related clue (we also find the number of clues found in total and a list containing the ids of the markers found, this helps to keep track of the collection of markers and clues and to understand if the robot vision is working efficiently)
  - on the right of terminal instead we can see the messages related to the robot_fsm node that reports the behaviors performed by the robot such as the movements between and in the rooms, or the hypotheses found

The video was speeded up because due to the poor performance of my pc it turned out to be particularly slow, and to also allow us to see the robot find at least one hypothesis (even if incorrect) starting from the beginning

## Terminal images

Below I add some of the most significant screenshots produced by the terminal during the simulation:

In these two images we see the messages produced by the robot_fsm node showing the robot inside a small room and after in a big room. As we can read in the small room (bathroom) the robot reaches the goal point and just performs a rotation on itself; in the bigger room (living-room) instead the robot applies different behaviors in order to have better visit the whole room, in fact it moves to four different points of the room and performs some rotations.

![small](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/small.png)

![big](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/big_room.png)

In these three images we see that the cameras have detected markers and thus found related hints. It is good to see that the markers found follow the logic of the system I have thought of in fact the camera on the end-effector finds the marker (id 36) located above the wall (camera1), the camera on the base finds the marker (id 15) located at the base of the wall (camera2), and the camera facing downwards on the inclined arm finds the marker(id 35) on the floor:

![1](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/cam_1.png) 

![2](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/cam_2.png) 

![3](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/cam_3.png)


Here finally we can see the screens where the robot states its hypothesis, as we can see if the hypothesis is the wrong one the robot resumes searching, if the hypothesis is the right one the game ends:

![wrong](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/wrong_screen.png)

![winner](https://github.com/piquet8/exp_ass3/blob/main/media_exp3/winner_screen.png)



# Working hypothesis and environment

For this project, I started from the assumptions of the previous assignment [exp_ass1](https://github.com/piquet8/exp_ass1), specifically the behaviors necessary for the robot to achieve the goal, which are: moving the robot to search and collect hints, processing the hints in the ontology and testing ready hypotheses, and finally testing the hypotheses found. Definitely in this project more than in the previous ones, the major concentration was given to the first aspect then to the search for hints and on the behaviors required to obtain them. Indeed, in this project the robot has to deal with a 'real' and complex environment where the hints (the markers) are never placed in the same place. This had to provide for flexible solutions with wider possibility of adaptation to changes in the surrounding environment. For moving the robot, I used the move_base module because it is a good planner for complex environments and I have used it before. For marker detection I used several expedients, I decided not to move the arm during the search because the robot is often in difficult environments such as small rooms where the movement space is relatively small and the arm movement could predict unintended collisions or misbehavior, also the acquisition of images taken from a moving camera are less reliable. Therefore, not having the arm movement to allow the robot to 'look' in different positions, I used 3 cameras placed at 3 different locations and with 3 different viewpoints: one camera is placed on the base of the robot to view markers located at the bottom, one camera is placed on the arm to view markers at the top, and one camera is placed on the robot's inclined arm to view markers located on the floor. Finally, it was sufficient to add a z-axis rotation of the robot, slow enough to avoid the previously mentioned problem on the images captured in motion, and a different room search strategy depending on the size of the room. 

## System's features

## System's limitations

## System's technical improvements

# Specific link to folders  
- Here you can find the documentation: [docs](https://github.com/piquet8/exp_ass3/tree/main/docs)
- Here you can find the media and diagram file: [media and diagram](https://github.com/piquet8/exp_ass3/tree/main/media_exp3)

# Authors and contacts
AUTHOR: Gianluca Piquet

CONTACT: gianlucapiquet8@gmail.com 
