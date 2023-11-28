#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import time
import random
import math
import matplotlib.pyplot as plt
import rospkg
import cv2

from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import Twist         #cmd_vel publisher
from sensor_msgs.msg import LaserScan       #2d lidar data
from std_srvs.srv import Empty              #reset gazebo world
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import matplotlib.pyplot as plt


rospack = rospkg.RosPack()
rospack_path = rospack.get_path("ros_project3")

total_action_taken_pub                      = rospy.Publisher('total_action_taken', Int64, queue_size=1)

action_forward_with_turn_little_left_pub    = rospy.Publisher('action_forward_with_turn_little_left', Float32, queue_size=1)
action_forward_with_turn_hard_left_pub    = rospy.Publisher('action_forward_with_turn_hard_left', Float32, queue_size=1)
action_forward_with_turn_veryhard_left_pub    = rospy.Publisher('action_forward_with_turn_veryhard_left', Float32, queue_size=1)

action_forward_pub                          = rospy.Publisher('action_forward', Float32, queue_size=1)

action_forward_with_turn_little_right_pub   = rospy.Publisher('action_forward_with_turn_little_right', Float32, queue_size=1)
action_forward_with_turn_hard_right_pub   = rospy.Publisher('action_forward_with_turn_hard_right', Float32, queue_size=1)
action_forward_with_turn_veryhard_right_pub   = rospy.Publisher('action_forward_with_turn_veryhard_right', Float32, queue_size=1)


episode_pub                                 = rospy.Publisher('episode', Int64, queue_size=1)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

HIT_DISTANCE_THRESHOLD = 0.24
CLOSE_THRESHOLD = 0.28
NORMAL_THRESHOLD = 0.32
FAR_THRESHOLD = 0.36
LOST_DISTANCE_THRESHOLD = 0.7

#HIT_DISTANCE_THRESHOLD = 0.22
#CLOSE_THRESHOLD = 0.24
##NORMAL_THRESHOLD = 0.27
#FAR_THRESHOLD = 0.33
#LOST_DISTANCE_THRESHOLD = 0.36

#THIS IS THE MIN FRONT RIGHTTTTTTTTTTTTTTTTTTTTTTTTTTTT 0.3674941658973694 = 0.37
#THIS IS THE MIN FRONTTTTTTTTTTTTTTTTTTTTTTTTT 1.7958569526672363
#THIS IS THE MIN RIGHTTTTTTTTTTTTTTTTTTTTTTTTTTTT 0.29718658328056335

nSTATES = 80
nACTIONS = 5

# Initialize all variables
range_front = []
range_right = []
range_left  = []
range_back = []
range_front_right = []

min_front = 0
i_front = 0
min_back=0
i_back=0
min_right = 0
i_right = 0

min_left = 0
i_left = 0

min_front_right=0
i_front_right=0 

front = ""
left = ""
right = ""
rightfront= ""
back = ""
state = 0

ACTIONS     = ['FORWARD_AND_TURN_HARD_LEFT','FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT','FORWARD_AND_TURN_HARD_RIGHT']

nSTATES    = 80

MAX_EPISODE = 600   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.8   # discount rate
ALPHA       = 0.2  # (1-alpha)
EPSILON     = 0.9

global flag 
#global back 

episode = 0
global epsilon
epsilon = 0.9
max_epsilon = EPSILON
min_epsilon = 0.01       
decay = 0.01         

total_training_rewards = 0
training_rewards = []  
epsilons = []
total_action_taken = 0
total_action_taken2 = 0
total_action_taken3 = 0

forward_with_turn_hard_left       = 0
forward_with_turn_little_left       = 0
forward                             = 0
forward_with_turn_little_right      = 0
forward_with_turn_hard_right      = 0
ratio_forward_action_taken                  = [] 
ratio_forward_action_taken2                  = [] 
ratio_forward_action_taken3                 = [] 


def build_q_table():
    global nSTATES
    global ACTIONS
    table = pd.DataFrame(
        np.zeros((nSTATES, len(ACTIONS)), dtype=float),
        columns=ACTIONS
    )
    return table

def actionDecision(observation, q_table):
    global x
    x= random.random()
    #action= 'FORWARD_AND_TURN_LITTLE_LEFT'
    if x > epsilon:
        state_action = q_table.loc[observation, :]
        action = np.random.choice(state_action[state_action == np.max(state_action)].index)
        print("THIS IS THE greedy CHOICE", action)

    else:
        #actionindex= math.trunc(y)
        actionindex = random.randrange(len(ACTIONS))
        action= ACTIONS[actionindex]
        print("THIS IS THE RANDOM CHOICEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE", action)
    return action

def move_robot(speed_linear_x, speed_angular_z):
    msg = Twist()
    msg.linear.x = speed_linear_x
    msg.angular.z = speed_angular_z
    cmd_vel_pub.publish(msg)
  
def move_robot_by_action(action):
    global forward_with_turn_hard_left
    global forward_with_turn_veryhard_left
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global forward_with_turn_hard_right
    global forward_with_turn_veryhard_right

    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.1, 1.0)
        rospy.sleep(0.5)

        forward_with_turn_little_left += 1
    elif action == 'FORWARD_AND_TURN_HARD_LEFT':
        move_robot(0.1, 1.5)
        rospy.sleep(0.5)

        forward_with_turn_hard_left += 1
    elif action == 'FORWARD':
        move_robot(0.1,0)
        rospy.sleep(0.5)

        forward += 1
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.1, -1.0)
        rospy.sleep(0.5)

        forward_with_turn_little_right += 1
    elif action == 'FORWARD_AND_TURN_HARD_RIGHT':
        move_robot(0.1, -1.5)
        rospy.sleep(0.5)

        forward_with_turn_hard_right += 1

def move_robot_by_action2(action):
    global forward_with_turn_hard_left
    global forward_with_turn_veryhard_left
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global forward_with_turn_hard_right
    global forward_with_turn_veryhard_right

    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.1, 0.7)
        rospy.sleep(1)

        move_robot(0.0, 0.0)
        rospy.sleep(1)
        forward_with_turn_little_left += 1

    elif action == 'FORWARD_AND_TURN_HARD_LEFT':
        move_robot(0.1, 1.5)
        rospy.sleep(1)

        move_robot(0.0, 0.0)
        rospy.sleep(1)

        forward_with_turn_hard_left += 1
    elif action == 'FORWARD':
        move_robot(0.1,0)
        rospy.sleep(1)

        move_robot(0.0, 0.0)
        rospy.sleep(1)

        forward += 1
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.1, -0.7)
        rospy.sleep(1)

        move_robot(0.0, 0.0)
        rospy.sleep(1)

        forward_with_turn_little_right += 1
    elif action == 'FORWARD_AND_TURN_HARD_RIGHT':
        move_robot(0.1, -1.5)
        rospy.sleep(1)

        move_robot(0.0, 0.0)
        rospy.sleep(1)

        forward_with_turn_hard_right += 1

def move_robot_by_action3(action):
    global forward_with_turn_hard_left
    global forward_with_turn_veryhard_left
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global forward_with_turn_hard_right
    global forward_with_turn_veryhard_right

    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.0,0.0)
        rospy.sleep(1)
        move_robot(0.1, 0.2)
        rospy.sleep(5)
        move_robot(0.0,0.0)
        rospy.sleep(1)
        forward_with_turn_little_left += 1

    elif action == 'FORWARD_AND_TURN_HARD_LEFT':
        move_robot(0.0,0.0)
        rospy.sleep(1)
        move_robot(0.1, 0.5)
        rospy.sleep(2)
        move_robot(0.0,0.0)
        rospy.sleep(1)

        forward_with_turn_hard_left += 1
    elif action == 'FORWARD':
        move_robot(0.0,0.0)
        rospy.sleep(1)
        move_robot(0.12,0)
        rospy.sleep(4)
        move_robot(0.0,0.0)
        rospy.sleep(1)

        forward += 1
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.0,0.0)
        rospy.sleep(1)    
        move_robot(0.12, -0.2)
        rospy.sleep(16)
        move_robot(0.05,0.0)
        rospy.sleep(1)
        move_robot(0.0,0.0)
        rospy.sleep(1)   
        forward_with_turn_little_right += 1
    elif action == 'FORWARD_AND_TURN_HARD_RIGHT':
        move_robot(0.0,0.0)
        rospy.sleep(1)
        move_robot(0.1, -0.5)
        rospy.sleep(2)
        move_robot(0.0,0.0)
        rospy.sleep(1)

        forward_with_turn_hard_right += 1

def init_env(episode, MAX_EPISODE):
    threshold = MAX_EPISODE / 3
    print(threshold)
    if episode <= threshold:
        world_x = 0.8
        world_y = -1.90
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1

    # if episode % 3 == 1:
    elif (threshold < episode <= (2 * threshold)):
        world_x = 1.74
        world_y = -1.95
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 0.0

    # if episode % 3 == 2:
    elif ((2 * threshold) <= episode <= MAX_EPISODE):
        world_x = 1.5
        world_y = -1.0
        world_orientation_x = 0
        world_orientation_y = 0
        world_orientation_z = 3.14
        world_orientation_w = 0

    #else:
    #    world_x = 0.1
    #    world_y = -2.0
    #    world_orientation_x = 0.0
    #    world_orientation_y = 0.0
    #    world_orientation_z = 0.0
    #    world_orientation_w = 1

    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_waffle_pi'
    state_msg.pose.position.x = world_x
    state_msg.pose.position.y = world_y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = world_orientation_x
    state_msg.pose.orientation.y = world_orientation_y
    state_msg.pose.orientation.z = world_orientation_z
    state_msg.pose.orientation.w = world_orientation_w
    try:
        gazebo_set_state_srv( state_msg )
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
    
    end = False #to start the new episode
    return end

def get_state(front, rightfront, right, back, left):
    global state
    end = False                                         
    
    if (front=="veryFar" and rightfront=="far" and right=="veryFar"):
        state = 0                                                                                   ## 
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="close" and left =="far"):
        state = 1                                                                           #This is the next step after turning left from state 32 (WRONG)
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "medium" and left=="medium"):
        state = 75                                                                                                      #initial position for learning feature 3 
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="far" and left== "medium"):
        state = 74                                                                                                  #initial position for learning feature 3
    
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "far" and left=="medium"):
        state = 77                                                                                                  #initial position for learning feature 3
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="medium" and left== "medium"):
        state = 78            
                                                                                              #initial position for learning feature 3
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="far" and left== "far"):
        state = 79                                                                                                      #final state after taking a u-turn

    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "far" and left=="far"):
        state = 2                                       
    elif (front=="veryFar" and rightfront=="far" and right=="close"):
        state = 3                                       
    elif (front=="veryFar" and rightfront=="far" and right=="veryClose"):               
        state = 4                                                                                   ##

    elif (front=="veryFar" and rightfront=="medium" and right=="veryFar"):
        state = 5                                                                                   ## 
    elif (front=="veryFar" and rightfront=="medium" and right=="far"):
        state = 6                                      
    elif (front=="veryFar" and rightfront=="medium" and right=="medium" and back=="far" and left=="far"):
        state = 7                                                                                       #FORWARD!

    elif (front=="veryFar" and rightfront=="medium" and right=="close"):
        state = 8                                       
    elif (front=="veryFar" and rightfront=="medium" and right=="veryClose"):
        state = 9                                                                                   ##   

    elif (front=="veryFar" and rightfront=="close" and right=="veryFar"):
        state = 10                                                                                      ## 
    elif (front=="veryFar" and rightfront=="close" and right=="far"):
        state = 11                                      
    elif (front=="veryFar" and rightfront=="close" and right=="medium" and back =="far" and left=="far"):
        state = 12                                       
    elif (front=="veryFar" and rightfront=="close" and right=="close"):
        state = 13                                       
    elif (front=="veryFar" and rightfront=="close" and right=="veryClose"):
        state = 14                                                                                  ## 

############

    elif (front=="far" and rightfront=="far" and right=="veryFar"):
        state = 15                                                                                      ## 
    elif (front=="far" and rightfront=="far" and right=="far" and back=="close"):
        state = 16                                      
    elif (front=="far" and rightfront=="far" and right=="medium" and left == "far"):
        state = 17                                       
    elif (front=="far" and rightfront=="far" and right=="close"):
        state = 18                                       
    elif (front=="far" and rightfront=="far" and right=="veryClose"):
        state = 19  

    elif (front=="far" and rightfront=="medium" and right=="veryFar"):
        state = 20                                                                                          ## 
    elif (front=="far" and rightfront=="medium" and right=="far"):
        state = 21                                      
    elif (front=="far" and rightfront=="medium" and right=="medium" and back =="far" and left=="far"):
        state = 22                                                                                          #FORWARD!
    elif (front=="far" and rightfront=="medium" and right=="close"):
        state = 23                                       
    elif (front=="far" and rightfront=="medium" and right=="veryClose"):
        state = 24                                                                                  ## 

    elif (front=="far" and rightfront=="close" and right=="veryFar"):
        state = 25                                                                                      ## 
    elif (front=="far" and rightfront=="close" and right=="far"):
        state = 26                                      
    elif (front=="far" and rightfront=="close" and right=="medium" and left== "far"): 
        state = 27                                       
    elif (front=="far" and rightfront=="close" and right=="close"):
        state = 28                                       
    elif (front=="far" and rightfront=="close" and right=="veryClose"):
        state = 29                                                                                  ## 

############

    elif (front=="medium" and rightfront=="far" and right=="veryFar"):
        state = 30                                                                                      ## 
    elif (front=="medium" and rightfront=="far" and right=="far"):
        state = 31                                      
    elif (front=="medium" and rightfront=="far" and right=="medium" and back=="far" and left=="medium"):
        state = 32  
                                                                                                #This is the state where it has to take a left                               
    elif (front=="medium" and rightfront=="far" and right=="close"):
        state = 33                                       
    elif (front=="medium" and rightfront=="far" and right=="veryClose"):
        state = 34                                                                                          ## 

    elif (front=="medium" and rightfront=="medium" and right=="veryFar"):
        state = 35                                                                                      ## 
    elif (front=="medium" and rightfront=="medium" and right=="far"):
        state = 36                                                                                 #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="medium"):
        state = 37                                                                                  #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="close" and back == "far" and left== "medium"):
        state = 38                                                                                  #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="veryClose"):
        state = 39                                                                                              ## 

    elif (front=="medium" and rightfront=="close" and right=="veryFar"):
        state = 40                                                                                            ## 
    elif (front=="medium" and rightfront=="close" and right=="far"):
        state = 41                                      
    elif (front=="medium" and rightfront=="close" and right=="medium"):
        state = 42                                       
    elif (front=="medium" and rightfront=="close" and right=="close"):
        state = 43                                       
    elif (front=="medium" and rightfront=="close" and right=="veryClose"):
        state = 44                                                                                                  ## 

###############

    elif (front=="close" and rightfront=="far" and right=="veryFar"):
        state = 45                                                                                           ## 
    elif (front=="close" and rightfront=="far" and right=="far"):
        state = 46
                                                              
    elif (front=="close" and rightfront=="far" and right=="medium"):
        state = 47   
                                                              
    elif (front=="close" and rightfront=="far" and right=="close"):
        state = 48  
                                                                   
    elif (front=="close" and rightfront=="far" and right=="veryClose"):
        state = 49                                                                                      ## 

    elif (front=="close" and rightfront=="medium" and right=="veryFar"):
        state = 50                                                                                        ## 
    elif (front=="close" and rightfront=="medium" and right=="far"):
        state = 51     
                                                               
    elif (front=="close" and rightfront=="medium" and right=="medium"):
        state = 52    
                            
    elif (front=="close" and rightfront=="medium" and right=="close"):
        state = 53  
                                                                   
    elif (front=="close" and rightfront=="medium" and right=="veryClose"):
        state = 54                                                                                              ## 
    elif (front=="close" and rightfront=="close" and right=="veryFar"):
        state = 55                                                                                              ## 
    elif (front=="close" and rightfront=="close" and right=="far"):
        state = 56      
                                                              
    elif (front=="close" and rightfront=="close" and right=="medium"):
        state = 57   
                                                                                             #$$
    elif (front=="close" and rightfront=="close" and right=="close"):
        state = 58     
                                                              
    elif (front=="close" and rightfront=="close" and right=="veryClose"):
        state = 59                                                                                          ## 

###############

    elif (front=="veryClose" and rightfront=="far" and right=="veryFar"):
        state = 60                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="far"):
        state = 61                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="medium"):
        state = 62                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="close"):
        state = 63                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="veryClose"):
        state = 64                                                                                      ##

    elif (front=="veryClose" and rightfront=="medium" and right=="veryFar"):
        state = 65                                                                                         ##
    elif (front=="veryClose" and rightfront=="medium" and right=="far"):
        state = 66                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="medium"):
        state = 67                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="close"):
        state = 68                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="veryClose"):
        state = 69                                                                                              ##

    elif (front=="veryClose" and rightfront=="close" and right=="veryFar"):
        state = 70                                                                                                   ##
    elif (front=="veryClose" and rightfront=="close" and right=="far"):
        state = 71                                                                                                   ##
    elif (front=="veryClose" and rightfront=="close" and right=="medium"):
        state = 72                                                                                                           ##
    elif (front=="veryClose" and rightfront=="close" and right=="close"):
        state = 73                                                                                                      ##
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "medium" and left=="medium"):
        state = 76                                                                                                            ##


    print("STATE ", state)


    if  state== 73 or state ==72 or state==71 or state ==70 or state==69 or state == 68 or state== 67 or state == 66 or state==65 or state ==64 or state==63 or state ==62 or state==61 or state ==60 or state== 59 or state == 55 or state ==54 or state ==50 or state ==49 or state ==45 or state ==44 or state == 40 or state ==39 or state == 35 or state ==34 or state ==30 or state ==29 or state ==25 or state ==24 or state ==20 or state ==15 or state ==14 or state ==10 or state ==9 or state ==5 or state ==4 or state ==0 :
        rospy.loginfo("Crashed the wall/Too far from the wall - Resetting Gazebo World")
        print(state)
        reward = -10
        end = True
    elif  state ==27 or state ==22 or state ==17 or state ==12 or state ==7 or state ==2:
        reward = 10
    
    elif state == 1:
        reward = 10
    #elif state == 79:
    #    reward = 10
    #elif state == 16:
    #    reward = 5
    else:
        reward=0
 
    return state, reward, end

def get_state_initial(front, rightfront, right, back, left ):
    global state

    if (front=="veryFar" and rightfront=="far" and right=="veryFar"):
        state = 0                                                                                   ## 
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="close" and left =="far"):
        state = 1                                                                           #This is the next step after turning left from state 32 (WRONG)
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "medium" and left=="medium"):
        state = 75                                                                                                      #initial position for learning feature 3 
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="far" and left== "medium"):
        state = 74                                                                                                  #initial position for learning feature 3
    
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "far" and left=="medium"):
        state = 77                                                                                                  #initial position for learning feature 3
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="medium" and left== "medium"):
        state = 78            
                                                                                              #initial position for learning feature 3
    elif (front=="veryFar" and rightfront=="far" and right=="far" and back =="far" and left== "far"):
        state = 79                                                                                                      #final state after taking a u-turn

    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "far" and left=="far"):
        state = 2                                       
    elif (front=="veryFar" and rightfront=="far" and right=="close"):
        state = 3                                       
    elif (front=="veryFar" and rightfront=="far" and right=="veryClose"):               
        state = 4                                                                                   ##

    elif (front=="veryFar" and rightfront=="medium" and right=="veryFar"):
        state = 5                                                                                   ## 
    elif (front=="veryFar" and rightfront=="medium" and right=="far"):
        state = 6                                      
    elif (front=="veryFar" and rightfront=="medium" and right=="medium" and back=="far" and left=="far"):
        state = 7                                                                                       #FORWARD!

    elif (front=="veryFar" and rightfront=="medium" and right=="close"):
        state = 8                                       
    elif (front=="veryFar" and rightfront=="medium" and right=="veryClose"):
        state = 9                                                                                   ##   

    elif (front=="veryFar" and rightfront=="close" and right=="veryFar"):
        state = 10                                                                                      ## 
    elif (front=="veryFar" and rightfront=="close" and right=="far"):
        state = 11                                      
    elif (front=="veryFar" and rightfront=="close" and right=="medium" and back =="far" and left=="far"):
        state = 12                                       
    elif (front=="veryFar" and rightfront=="close" and right=="close"):
        state = 13                                       
    elif (front=="veryFar" and rightfront=="close" and right=="veryClose"):
        state = 14                                                                                  ## 

############

    elif (front=="far" and rightfront=="far" and right=="veryFar"):
        state = 15                                                                                      ## 
    elif (front=="far" and rightfront=="far" and right=="far"):
        state = 16                                      
    elif (front=="far" and rightfront=="far" and right=="medium" and left == "far"):
        state = 17                                       
    elif (front=="far" and rightfront=="far" and right=="close"):
        state = 18                                       
    elif (front=="far" and rightfront=="far" and right=="veryClose"):
        state = 19  

    elif (front=="far" and rightfront=="medium" and right=="veryFar"):
        state = 20                                                                                          ## 
    elif (front=="far" and rightfront=="medium" and right=="far"):
        state = 21                                      
    elif (front=="far" and rightfront=="medium" and right=="medium" and back =="far" and left=="far"):
        state = 22                                                                                          #FORWARD!
    elif (front=="far" and rightfront=="medium" and right=="close"):
        state = 23                                       
    elif (front=="far" and rightfront=="medium" and right=="veryClose"):
        state = 24                                                                                  ## 

    elif (front=="far" and rightfront=="close" and right=="veryFar"):
        state = 25                                                                                      ## 
    elif (front=="far" and rightfront=="close" and right=="far"):
        state = 26                                      
    elif (front=="far" and rightfront=="close" and right=="medium" and left== "far"): 
        state = 27                                       
    elif (front=="far" and rightfront=="close" and right=="close"):
        state = 28                                       
    elif (front=="far" and rightfront=="close" and right=="veryClose"):
        state = 29                                                                                  ## 

############

    elif (front=="medium" and rightfront=="far" and right=="veryFar"):
        state = 30                                                                                      ## 
    elif (front=="medium" and rightfront=="far" and right=="far"):
        state = 31                                      
    elif (front=="medium" and rightfront=="far" and right=="medium" and back=="far" and left=="medium"):
        state = 32  
                                                                                                #This is the state where it has to take a left                               
    elif (front=="medium" and rightfront=="far" and right=="close"):
        state = 33                                       
    elif (front=="medium" and rightfront=="far" and right=="veryClose"):
        state = 34                                                                                          ## 

    elif (front=="medium" and rightfront=="medium" and right=="veryFar"):
        state = 35                                                                                      ## 
    elif (front=="medium" and rightfront=="medium" and right=="far"):
        state = 36                                                                                 #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="medium"):
        state = 37                                                                                  #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="close" and back == "far" and left== "medium"):
        state = 38                                                                                  #TURN LEFT!
    elif (front=="medium" and rightfront=="medium" and right=="veryClose"):
        state = 39                                                                                              ## 

    elif (front=="medium" and rightfront=="close" and right=="veryFar"):
        state = 40                                                                                            ## 
    elif (front=="medium" and rightfront=="close" and right=="far"):
        state = 41                                      
    elif (front=="medium" and rightfront=="close" and right=="medium"):
        state = 42                                       
    elif (front=="medium" and rightfront=="close" and right=="close"):
        state = 43                                       
    elif (front=="medium" and rightfront=="close" and right=="veryClose"):
        state = 44                                                                                                  ## 

###############

    elif (front=="close" and rightfront=="far" and right=="veryFar"):
        state = 45                                                                                           ## 
    elif (front=="close" and rightfront=="far" and right=="far"):
        state = 46
                                                              
    elif (front=="close" and rightfront=="far" and right=="medium"):
        state = 47   
                                                              
    elif (front=="close" and rightfront=="far" and right=="close"):
        state = 48  
                                                                   
    elif (front=="close" and rightfront=="far" and right=="veryClose"):
        state = 49                                                                                      ## 

    elif (front=="close" and rightfront=="medium" and right=="veryFar"):
        state = 50                                                                                        ## 
    elif (front=="close" and rightfront=="medium" and right=="far"):
        state = 51     
                                                               
    elif (front=="close" and rightfront=="medium" and right=="medium"):
        state = 52    
                            
    elif (front=="close" and rightfront=="medium" and right=="close"):
        state = 53  
                                                                   
    elif (front=="close" and rightfront=="medium" and right=="veryClose"):
        state = 54                                                                                              ## 
    elif (front=="close" and rightfront=="close" and right=="veryFar"):
        state = 55                                                                                              ## 
    elif (front=="close" and rightfront=="close" and right=="far"):
        state = 56      
                                                              
    elif (front=="close" and rightfront=="close" and right=="medium"):
        state = 57   
                                                                                             #$$
    elif (front=="close" and rightfront=="close" and right=="close"):
        state = 58     
                                                              
    elif (front=="close" and rightfront=="close" and right=="veryClose"):
        state = 59                                                                                          ## 

###############

    elif (front=="veryClose" and rightfront=="far" and right=="veryFar"):
        state = 60                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="far"):
        state = 61                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="medium"):
        state = 62                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="close"):
        state = 63                                                                                      ##
    elif (front=="veryClose" and rightfront=="far" and right=="veryClose"):
        state = 64                                                                                      ##

    elif (front=="veryClose" and rightfront=="medium" and right=="veryFar"):
        state = 65                                                                                         ##
    elif (front=="veryClose" and rightfront=="medium" and right=="far"):
        state = 66                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="medium"):
        state = 67                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="close"):
        state = 68                                                                                          ##
    elif (front=="veryClose" and rightfront=="medium" and right=="veryClose"):
        state = 69                                                                                              ##

    elif (front=="veryClose" and rightfront=="close" and right=="veryFar"):
        state = 70                                                                                                   ##
    elif (front=="veryClose" and rightfront=="close" and right=="far"):
        state = 71                                                                                                   ##
    elif (front=="veryClose" and rightfront=="close" and right=="medium"):
        state = 72                                                                                                           ##
    elif (front=="veryClose" and rightfront=="close" and right=="close"):
        state = 73                                                                                                      ##
    elif (front=="veryFar" and rightfront=="far" and right=="medium" and back == "medium" and left=="medium"):
        state = 76          


    return state

def Qlearn():
    global left
    global front
    global right
    global back 
    global rightfront
    global total_training_rewards
    global total_action_taken
    global total_action_taken2
    global total_action_taken3

    global episode
    global epsilon
    global state
    global f
    global x
    global count
    f=0
    global steps
    steps=0
    global g
    g=0
    global h 
    h=0
                                        
    q_table = build_q_table()
    episode = 0

    while episode < (MAX_EPISODE):
        if rospy.is_shutdown():
            break
        
        end = init_env(episode, MAX_EPISODE)
        reward = 0

        state = get_state_initial(front, rightfront, right, back, left)  
        print("This is the very initial state before the inner loop", state)
        steps=0
        if episode < (MAX_EPISODE/3):
            y=10
        elif (MAX_EPISODE/3) < episode < ((MAX_EPISODE/3)*2):
            y=1
        elif ((MAX_EPISODE/3)*2) < episode < MAX_EPISODE:
            y=1

        while not end and not rospy.is_shutdown() and steps <y:                  
            steps+=1
            print("the number of steps taken in this episode is ", steps)
            print(state)

            act = actionDecision(state, q_table)
            #act= 'FORWARD_AND_TURN_LITTLE_RIGHT'
            
            #unpause phy
            try:
                unpause_physics_client()
            except rospy.ServiceException as e:
                print (" unpause physics Service call failed: %s" % e)
    
            if episode < (MAX_EPISODE/3) :
                move_robot_by_action(act)
                rospy.sleep(0.4)

            if (MAX_EPISODE/3) < episode < ((MAX_EPISODE/3)*2):
                rospy.sleep(1)
                move_robot_by_action2(act)

            if ((MAX_EPISODE/3)*2) < episode < MAX_EPISODE:
                move_robot_by_action3(act)
                rospy.sleep(0.4)

            #pause phy
            try:
                pause_physics_client()
            except rospy.ServiceException as e:
                print (" pause physics Service call failed: %s" % e)

            ################### Calculating the ratio of correct forward actions during learning following a straight wall feature (DONE AND VERIFIED 02/25/23)
            if episode < MAX_EPISODE/3:
                
                if x > epsilon and ((state==7 and act == 'FORWARD') or (state==2 and act=='FORWARD') ):
                    f+=1
                    total_action_taken += 1

                elif x > epsilon and ((state==7) or (state==2)):
                    total_action_taken += 1     
                
                print("the total value of f is", f)
                print("the total actions taken are", total_action_taken)

            ################### Calcularing the ratio of correct left turns during learning taking left turns feature (DONE AND VERIFIED 02/25/23)
            if MAX_EPISODE/3 < episode < ((MAX_EPISODE/3)*2):
                
                if x > epsilon and (((state==32) or (state==31) or (state==1) or (state==38)) and act=='FORWARD_AND_TURN_HARD_LEFT') :
                    g+=1
                    total_action_taken2 += 1

                elif x > epsilon:
                    total_action_taken2 += 1  

                print("the total value of g is", g)
                print("the total actions taken are", total_action_taken2)
            
            ################### Calculating the ratio of correct u-turns during learning the u-turns feature 
            if episode > ((MAX_EPISODE/3)*2):

                if x > epsilon and (((state == 75) or (state==74) or (state==77) or (state==78) or (state==79) or (state==16)) and act=='FORWARD_AND_TURN_LITTLE_RIGHT') :
                    h+=1
                    total_action_taken3 += 1

                elif x > epsilon and ((state!=75) or (state!=74) or (state!=77) or (state!=78) or (state!=79)):
                    total_action_taken3 += 1 

                print("the total value of h is", h)
                print("the total actions taken are", total_action_taken3)

            ####################

            next_state, reward, end = get_state(front, rightfront, right, back, left)

            print("the value of x is:")    
            print (x)
            print("the value of epsilon is:")
            print(epsilon)

            q_current = q_table.loc[state, act]

            q_target = reward + (GAMMA * q_table.iloc[next_state].max())

            q_table.loc[state, act] += ALPHA * (q_target - q_current)

            state = next_state

        if episode == 101 or episode == 201:
            epsilon=0.9

        if episode <MAX_EPISODE/3:
            epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay*(episode))
            print(epsilon)

        if MAX_EPISODE/3 < episode < ((MAX_EPISODE/3)*2):
            epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay*(episode-201))
            print(epsilon)

        if ((MAX_EPISODE/3)*2) < episode < MAX_EPISODE:
            epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay*(episode-401))
            print(epsilon)      
            
            
        if total_action_taken != 0:
            ratio_forward_action_taken.append(f/total_action_taken)
        if total_action_taken2 != 0:
            ratio_forward_action_taken2.append(g/total_action_taken2)
        if total_action_taken3 != 0:
            ratio_forward_action_taken3.append(h/total_action_taken3)

        episode = episode +1
        save_to_filename = "%s/Q_Table_episode_%d.csv" % (rospack_path, episode)
        q_table.to_csv(save_to_filename)    

        filename = rospack_path + "/graph/graph.png"
        fig, ax = plt.subplots() #Creates a matplotlib subplot
        #Plots the given data
        print("ratio_forward_action_taken: ", ratio_forward_action_taken)
        print("ratio_forward_action_taken2: ", ratio_forward_action_taken2)
        print("ratio_forward_action_taken3: ", ratio_forward_action_taken3)
        ax.plot(range(len(ratio_forward_action_taken)), ratio_forward_action_taken, color='r')
        ax.plot(range(len(ratio_forward_action_taken2)), ratio_forward_action_taken2, color='g')
        ax.plot(range(len(ratio_forward_action_taken3)), ratio_forward_action_taken3, color='b')
        fig.savefig(filename)
        plt.close()
        show_graph(filename)

    plt.figure(figsize=(10,6))
    plt.plot(range(len(ratio_forward_action_taken)), ratio_forward_action_taken, color='r')
    plt.plot(range(len(ratio_forward_action_taken2)), ratio_forward_action_taken2, color='g')
    plt.plot(range(len(ratio_forward_action_taken3)), ratio_forward_action_taken3, color='b')

    plt.xlabel('Episode')
    plt.ylabel('Correct actions taken ratio')
    plt.title('Ratio of correct actions, red= fwd, green= left, blue= u-turn')
    plt.show()
    
    print (q_table)

    #xx = range(episode)
    #plt.figure(figsize=(10,6))

    #plt.plot(range(len(ratio_forward_action_taken)), ratio_forward_action_taken)
    #plt.xlabel('Episode')
    #plt.ylabel('Ratio Forward action taken')
    #plt.title('Ratio Forward action taken in all episodes') 
    #plt.show()    
    return q_table

def show_graph(filename):
    image = cv2.imread(filename)
    window_name = "Graph"
    cv2.imshow(window_name, image)
    cv2.waitKey(25)

def laserscan_callback(msg):
    global reward
    global prior_state_
    global state

    global range_front
    global range_right
    global range_left
    global range_front_right
    
    global range_back
    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left, min_front_right, i_front_right, min_back, i_back
    global back

    global front
    global right
    global left
    global rightfront
    
    ranges = msg.ranges
    
    # get the range FOV (Field of View)
    range_front[:15] = msg.ranges[15:0:-1]  # Front1 FOV (between 5 to -5 degrees)
    range_front[15:] = msg.ranges[-1:-15:-1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[270:290]  # right FOV (between 300 to 345 degrees)
    range_front_right= msg.ranges[291:320]
    range_left=msg.ranges[85:95]
    range_back= msg.ranges [179:181]

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_front_right,i_front_right = min( (range_front_right[i_front_right],i_front_right) for i_front_right in range(len(range_front_right)) )
    min_back, i_back= min((range_back[i_back], i_back)for i_back in range (len(range_back)))

    min_left,i_left = min( (range_left[i_left],i_left) for i_left in range(len(range_left)) )
    
    #Left states 

    if (min_left <= 1.25):
        left= "close"
    elif (1.25 <= min_left < 1.5):
        left = "medium"
    elif ( 1.5 <= min_left):
        left= "far"  
    
    #back states
 
    min_back, i_back= min((range_back[i_back], i_back)for i_back in range (len(range_back)))
    if (min_back <= 0.47):
        back= "close"
    elif (0.47 <= min_back < 0.8):
        back = "medium"
    elif ( 0.8 <= min_back):
        back= "far"

     #front states
    if (0.25< min_front< 0.44):
        front="veryClose"
    elif (0.44 <= min_front < 0.60):
        front = "close"
    elif (0.60 <= min_front < 0.70):
        front = "medium"
    elif (0.70 <= min_front < 0.75):
        front = "far"
    elif (0.75 <= min_front):
        front= "veryFar"

     #right states
    if (0.2<= min_right< 0.38):
        right="veryClose"
    elif (0.38 <= min_right < 0.42):
        right = "close"
    elif (0.42 <= min_right < 0.46):
        right = "medium"
    elif (0.46 <= min_right < 0.64):
        right = "far"
    elif (0.64 <= min_right):
        right= "veryFar"

#The below was done for feature 1 training
 #   if (0.2<= min_right< 0.38):
 #       right="veryClose"
 #   elif (0.38 <= min_right < 0.42):
 #       right = "close"
 #   elif (0.42 <= min_right < 0.46):
#        right = "medium"
#    elif (0.46 <= min_right < 0.5):
#        right = "far"
#    elif (0.5 <= min_right):
#        right= "veryFar"

     #right states
  #  if (0.2<= min_right< 0.33):
  #      right="veryClose"
  #  elif (0.33 <= min_right < 0.37):
  #      right = "close"
  #  elif (0.37 <= min_right < 0.49):
  #      right = "medium"
  #  elif (0.49 <= min_right < 0.7):
  #      right = "far"
  #  elif (0.7 <= min_right):
  #      right= "veryFar"


    #right front states
    if (0.25 < min_front_right <= 0.30):
        rightfront = "close"
    elif (0.30 < min_front_right <= 0.45):
        rightfront = "medium"
    elif (min_front_right > 0.45):
        rightfront = "far"

if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service('/gazebo/set_model_state')

    gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
    unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)

    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    rospy.sleep(3.0)

    q_table = Qlearn()
    # rospy.spin()
    print("Q TABLE")
    # print(q_table)
    print(" ")