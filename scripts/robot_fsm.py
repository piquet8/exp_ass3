#! /usr/bin/env python

## @package exp_ass3
#
#  \file robot_fsm.py
#  \brief this file implements the finite state machine that controls the robot behavior
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 30/09/2022
#  \details
#  
#  Subscribes to: <BR>
#   /hypothesis
#    
#  Publishes to: <BR>
#   /cmd_vel
#    
#  Services: <BR>  
# 
#  Action Services: <BR>
#   move_base
#
#  Client Services: <BR>
#   /oracle_solution 
#    
#
#  Description: <BR>
# In this function the finite state machine that controls the behavior of the robot is implemented, specifically there are three states:
# - state 0 -> goto_room: in this state we have the robot moving from one room to another, the room is chosen randomly and its coordinates are set as the goal 
# for the move_base module. The function is designed so that all rooms are seen randomly at each turn. When the robot reaches the room it switches to state 1
# - state 1 -> look_around: in this state the robot is inside the room and performs certain maneuvers to search for clues around it, if 
# the room is small the robot just turns on itself 360 degrees, if on the other hand the room is large the robot in addition to turning on itself moves to different
# points in the room. When the robot finishes the search if no hypothesis has been found I switch back to state 0
# - state 2 -> go_home in this last state the robot has found a complete hypothesis to test, it then goes to the home position and states its hypothesis to check if 
# it is the correct one. If the hypothesis is correct the game ends otherwise the robot resumes the search returning to state 0
# In both state 0 and state 1 if a complete hypothesis is detected the robot goes directly to state 2
#

import rospy
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Twist, Point, Pose
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
from math import pi
from std_msgs.msg import String, Int32, Bool
from erl2.srv import Oracle
from exp_ass3.srv import *
from exp_ass3.msg import *

#global variables
finish_game=0
goal=MoveBaseGoal()
client=None
rooms=[ [-4,-3],[-4,2],[-1,7.5],[5,-7],[5,-3], [4,1]]
visited=[]
pub_=None
state=1
ready=False
start = 0

id=None
who=None
what=None
where=None
move_arm=None
room_name = None
n_room = 0

##
#   \brief turn_around function
#   \param : 
#   \return : None
#   
#   This function implements the rotation of the robot on its z axis

def turn_around():
    print("Im looking aound me...")
    global pub_
    vel=Twist()
    vel.angular.z=0.8
    pub_.publish(vel)
    time.sleep(80)
    vel.angular.z=0

##
#   \brief cancel_function function
#   \param : 
#   \return : None
#   
#   This function its called when a new hypothesis is ready, it cancels the current goal and set the valute of the state to 2

def cancel_function():
    global ready, state
    client.cancel_goal()
    state = 2

##
#   \brief rooms function
#   \param : x,y
#   \return : None
#   
#   This function given as input the target coordinate values determines the name of the room

def _rooms(x,y):
    global room_name
    if x==-4 and y==-3:
        room_name = "Kitchen"
    elif x == -4 and y == 2:
        room_name = "Hall"
    elif x == -1 and y == 7.5:
        room_name = "Living-room"
    elif x == 5 and y == -7:
        room_name = "Bedroom"
    elif x==5 and y == -3:
        room_name = "Bathroom"
    elif x == 4 and y ==1:
        room_name = "Dining-room"

##
#   \brief goto_room function
#   \param : 
#   \return : None
#   
#   This function allows the robot to move from room to another one by randomly generating the room to be reached

def goto_room():
    global goal, rooms, visited, state, pub_, ready, room_name, n_room
    print("--------------------------------")
    start = 1
    if ready == True:
        state = 2
    else:
        #find random room to visit
        prev_=1
        while prev_== 1:
            room = random.randint(0, 5)
            if room in visited:
                prev_ = 1
            else:
                prev_ = 0       
        #print("Rooms visited so far: ")
        #n_room = n_room + 1
        #print(n_room)
        _rooms(rooms[room][0],rooms[room][1])
        print("Im going to new room..")
        print(room_name)
        #set the goal for move_base
        goal.target_pose.pose.position.x=rooms[room][0]
        goal.target_pose.pose.position.y=rooms[room][1]
        vel = Twist()
        vel.linear.x=0.5
        vel.linear.y=0.5
        pub_.publish(vel)
        client.send_goal(goal)
        if ready == True:
            cancel_function()
        else:
            client.wait_for_result()
        visited.append(room)
        #print(visited)
        if len(visited)== 6:
            print("All rooms visited! Start again the tour..")
            #empty the list of rooms visited
            visited.clear()
        state=1

##
#   \brief look_around function
#   \param : 
#   \return : None
#   
#   This function deals with room scouting, it implements different movements and behaviors according to the room the robot is in

def look_around():
     global pub_,state,hyp, ready, start, room_name
     print("--------------------------------")
     print("Im looking for hints:")
     if start == 1:
        if room_name == "Living-room":
            print("This room is really big, I will need more time to look for the hints here")
            goal.target_pose.pose.position.x=-5
            goal.target_pose.pose.position.y=7.5
            client.send_goal(goal)
            client.wait_for_result()

            print("Let's try this way too..")
            goal.target_pose.pose.position.x=-5
            goal.target_pose.pose.position.y=5
            client.send_goal(goal)
            if ready==True:
                cancel_function()
            else:
                client.wait_for_result()

            print("Let's try this way too..")
            goal.target_pose.pose.position.x=-1
            goal.target_pose.pose.position.y=5
            client.send_goal(goal)
            if ready==True:
                cancel_function()
            else:
                client.wait_for_result()
                print('I didn t find hypothesis yet. Lets go in another room!')
                state = 0

        elif room_name == "Kitchen":
            print("This room is really big, I will need more time to look for the hints here")
            turn_around()
            goal.target_pose.pose.position.x=0
            goal.target_pose.pose.position.y=-3.5
            client.send_goal(goal)
            turn_around()
            if ready==True:
                cancel_function() 
            else:
                client.wait_for_result() 
                print('I didn t find hypothesis yet. Lets go in another room!')
                state = 0

        elif room_name == "Dining-room":
            print("I will need more time to look for the hints here, they are hidden well")
            goal.target_pose.pose.position.x=5
            goal.target_pose.pose.position.y=2
            client.send_goal(goal)
            client.wait_for_result()
            turn_around()

            print("Let's try this way too..")
            goal.target_pose.pose.position.x=5
            goal.target_pose.pose.position.y=-0.5
            client.send_goal(goal)
            if ready==True:
                cancel_function()
            else:
                client.wait_for_result()
                print('I didn t find hypothesis yet. Lets go in another room!')
                state = 0

        else:   
            if ready == True:
                state = 2
            else:
                turn_around()
                print('Im checking if a new hypothesis is ready..')
                if ready == True:
                    state = 2
                else:   
                    print('I didn t find hypothesis yet. Lets go in another room!')
                    state = 0
     else:
        start = 1
        state = 0

##
#   \brief check_hyp function
#   \param : hyp
#   \return : None
#   
#   This function takes from the topic /oracle_hint the information about the detected hints and assigns it to the corresponding variables

def check_hyp(hyp):
    global state, id, who, what, where,ready
    ready= hyp.ready
    id=hyp.ID
    who=hyp.who
    what=hyp.what
    where=hyp.where

##
#   \brief go_home function
#   \param : 
#   \return : None
#   
#   This function moves the robot into the home to test its hypothesis

def go_home():
     global ready
     print("--------------------------------")
     print('Im ready! Lets go to the oracle!')
     goal.target_pose.pose.position.x=0
     goal.target_pose.pose.position.y=-2
     client.send_goal(goal)
     client.wait_for_result()
     ready = False
     check_win()

##
#   \brief check_win function
#   \param : 
#   \return : None
#   
#   This function checks whether the hypothesis found is the winning hypothesis

def check_win():
    global win_id, state, id, who, what, where, finish_game
    print("The hypotheis is: ")
    print(who + " with the " + what + " in the "+ where)
    win_id = oracle_solution()
    id_win = str(win_id)
    winner = id_win[4]
    candidate = id[2]
    if winner == candidate:
        print("RIGHT, YOU WIN!")
        finish_game=1
    else:
        print("WRONG! Try again")
        state = 0

##
#   \brief main function
#   \param : 
#   \return : None
#   
#   This function initilise the node and the relative sub/pub and clients

def main():
    global pub_, client, goal, finish_game, state, oracle_solution
    rospy.init_node('finite_state_machine')
    print('Ready to play the game!')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.Subscriber('/hypothesis', NewHyp, check_hyp)
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    oracle_solution = rospy.ServiceProxy('/oracle_solution', Oracle)
    goal.target_pose.header.frame_id="map"
    goal.target_pose.pose.orientation.w=1
    random.seed= int(time.time() * 1000)
    # while finish game variable is not 1 the robot will continue to move to reach the goal
    while finish_game==0:
        if state==0:
            goto_room()
        elif state==1:
            look_around()
        elif state==2:
            go_home()

if __name__ == '__main__':
    main()