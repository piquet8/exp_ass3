#! /usr/bin/env python

## @package exp_ass3
#
#  \file robot_vision.py
#  \brief this file implements 'the eyes' of the robot
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 30/09/2022
#  \details
#  
#  Subscribes to: <BR>
#   /camera1_publish/camera1_found_id
#   /camera2_publish/camera2_found_id
#   /camera3_publish/camera3_found_id
#    
#  Publishes to: <BR>
#   /new_hint
#    
#  Services: <BR>  
# 
#  Action Services: <BR>
#
#  Client Services: <BR>
#   /oracle_hint
#    
#
#  Description: <BR>
#  This file implements vision of the robot, here in fact are implemented 3 functions related to the three cameras on the robot that allow the 
#  acquisition of hints
#

import rospy
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
from exp_ass3.srv import Marker, MarkerResponse
from exp_ass3.msg import NewHint
from std_msgs.msg import String, Int32, Bool
import time
import random

#global variables
id_list=[]
oracle_service = None
pub_hint=rospy.Publisher('/new_hint', NewHint, queue_size = 10)
c_tot = 0

##
#   \brief cam1_newId
#   \param : new_id
#   \return : 
#   
#   This function takes from the topic /camera1_publisher/camera1_found_id the value of the id corresponded to the marker detected by camera1 
#   and uses it to ask the topic /oracle_hint which hints the id found corresponds to. This message is then processed and turned into a unique 
#   string that is posted to the topic /new_hint 

def cam1_newId (new_id):
    global oracle_service, pub_hint, c1, c_tot
    found=0
    req=Marker()
    for x in id_list:
        if (x==new_id.data):
            found=1
    if found==0:
        id_list.append(new_id.data)
        if (new_id.data>10) and (new_id.data<41):
            print("Camera 1 detect id: ")
            print(new_id.data)
            req=new_id.data
            hint=oracle_service(req)
            print(hint)
            c_tot = c_tot + 1 
            print("-----")
            print("Total number of hint found so far: ")
            print(c_tot)
            print("-----")
            print(id_list)
            if(hint.oracle_hint.key != '' and hint.oracle_hint.value != "-1"):
                new_hint = "ID" + str(hint.oracle_hint.ID) + ":" + hint.oracle_hint.key + ":" + hint.oracle_hint.value
                print(new_hint)
                pub_hint.publish(new_hint)
            else:
                print("FORMAT HINT IS WRONG!")
     
        else:
            print("id out of range")

##
#   \brief cam2_newId
#   \param : new_id
#   \return : 
#   
#   This function takes from the topic /camera2_publisher/camera2_found_id the value of the id corresponded to the marker detected by camera2 
#   and uses it to ask the topic /oracle_hint which hints the id found corresponds to. This message is then processed and turned into a unique 
#   string that is posted to the topic /new_hint 

def cam2_newId (new_id):
    global oracle_service, pub_hint, c_tot
    found=0
    req=Marker()
    for x in id_list:
        if (x==new_id.data):
            found=1
    if found==0:
        id_list.append(new_id.data)
        if (new_id.data>10) and (new_id.data<41):
            print("Camera 2 detect id: ")
            print(new_id.data)
            req=new_id.data
            hint=oracle_service(req)
            print(hint)
            c_tot = c_tot + 1
            print("-----")
            print("Total number of hint found so far: ")
            print(c_tot)
            print("-----")
            print(id_list)
            if(hint.oracle_hint.key != '' and hint.oracle_hint.value != "-1"):
                new_hint = "ID" + str(hint.oracle_hint.ID) + ":" + hint.oracle_hint.key + ":" + hint.oracle_hint.value
                print(new_hint)
                pub_hint.publish(new_hint)
            else:
                print("FORMAT HINT IS WRONG!")
        else:
            print("id out of range")

##
#   \brief cam3_newId
#   \param : new_id
#   \return : 
#   
#   This function takes from the topic /camera3_publisher/camera3_found_id the value of the id corresponded to the marker detected by camera3 
#   and uses it to ask the topic /oracle_hint which hints the id found corresponds to. This message is then processed and turned into a unique 
#   string that is posted to the topic /new_hint 

def cam3_newId (new_id):
    global oracle_service, pub_hint, c_tot
    found=0
    req=Marker()
    for x in id_list:
        if (x==new_id.data):
            found=1
    if found==0:
        id_list.append(new_id.data)
        if (new_id.data>10) and (new_id.data<41):    
            print("Camera 3 detect id: ")
            print(new_id.data)
            req=new_id.data
            hint=oracle_service(req)
            print(hint)
            c_tot = c_tot + 1
            print("-----")
            print("Total number of hint found so far: ")
            print(c_tot)
            print("-----")
            print(id_list)
            if(hint.oracle_hint.key != '' and hint.oracle_hint.value != "-1"):
                new_hint = "ID" + str(hint.oracle_hint.ID) + ":" + hint.oracle_hint.key + ":" + hint.oracle_hint.value
                print(new_hint)
                pub_hint.publish(new_hint)
            else:
                print("FORMAT HINT IS WRONG!")
        else:
            print("id out of range")

##
#   \brief main function
#   \param : 
#   \return : 
#   
#   This function initialise the node and all the pub/sub and clients 

def main():
    global pub_hint, oracle_service
    rospy.init_node('aruco')
    oracle_service = rospy.ServiceProxy('oracle_hint', Marker)
    rospy.Subscriber("/camera1_publish/camera1_found_id", Int32, cam1_newId)
    rospy.Subscriber("/camera2_publish/camera2_found_id", Int32, cam2_newId)
    rospy.Subscriber("/camera3_publish/camera3_found_id", Int32, cam3_newId)
    print("start the vision")
    rospy.spin() 

if __name__ == '__main__':
    main()
