#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import time
import random
import math
import matplotlib.pyplot as plt
import rospkg

from std_msgs.msg import Float32, Int64
from geometry_msgs.msg import Twist         #cmd_vel publisher
from sensor_msgs.msg import LaserScan       #2d lidar data
from std_srvs.srv import Empty              #reset gazebo world
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

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

HIT_DISTANCE_THRESHOLD = 0.15 
CLOSE_THRESHOLD = 0.20
NORMAL_THRESHOLD = 0.30
FAR_THRESHOLD = 0.40
LOST_DISTANCE_THRESHOLD = 0.45

nSTATES = 75
nACTIONS = 5

# Initialize all variables
range_front = []
range_right = []
range_left  = []
range_front_right = []

min_front = 0
i_front = 0

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
state = 0

ACTIONS     = ['FORWARD_AND_TURN_HARD_LEFT','FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT','FORWARD_AND_TURN_HARD_RIGHT']

nSTATES    = 75

MAX_EPISODE = 300   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.8   # discount rate
ALPHA       = 0.2  # (1-alpha)
EPSILON     = 0.9

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
forward_with_turn_hard_left       = 0
forward_with_turn_little_left       = 0
forward                             = 0
forward_with_turn_little_right      = 0
forward_with_turn_hard_right      = 0
ratio_forward_action_taken                  = [] 
ratio_forward_lil_left_action_taken          = []  
ratio_forward_hard_left_action_taken          = []  

ratio_forward_lil_right_action_taken          = []  
ratio_forward_hard_right_action_taken          = []  

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

    if x > epsilon:
        state_action = q_table.loc[observation, :]
        action = np.random.choice(state_action[state_action == np.max(state_action)].index)
        print("THIS IS THE greedy CHOICE", action)

    else:
        actionindex= math.trunc(x*5)
        actionindex = random.randrange(len(ACTIONS))
        action= ACTIONS[actionindex]
        print("THIS IS THE RANDOM CHOICE", action)
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
        move_robot(0.04, 0.05)
        forward_with_turn_little_left += 1
    elif action == 'FORWARD_AND_TURN_HARD_LEFT':
        move_robot(0.04, 0.25)
        forward_with_turn_hard_left += 1
    elif action == 'FORWARD':
        move_robot(0.04,0)
        forward += 1
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.04, -0.05)
        forward_with_turn_little_right += 1
    elif action == 'FORWARD_AND_TURN_HARD_RIGHT':
        move_robot(0.04, -0.15)
        forward_with_turn_hard_right += 1

def publish_all():
    global episode
    global total_action_taken
    global forward_with_turn_little_left
    global forward_with_turn_hard_left
    global forward_with_turn_veryhard_left

    global forward
    global forward_with_turn_little_right
    global forward_with_turn_hard_right
    global forward_with_turn_veryhard_right

    episode_msg = Int64()
    episode_msg.data = episode
    episode_pub.publish(episode_msg)

    action_msg = Int64()
    action_msg.data = total_action_taken
    total_action_taken_pub.publish(action_msg)

    msg = Float32()
    if total_action_taken!=0:
        msg.data = forward_with_turn_little_left        / total_action_taken
        action_forward_with_turn_little_left_pub.publish(msg)

        msg.data = forward_with_turn_hard_left        / total_action_taken
        action_forward_with_turn_hard_left_pub.publish(msg)

        msg.data = forward_with_turn_veryhard_left        / total_action_taken
        action_forward_with_turn_veryhard_left_pub.publish(msg)

        msg.data = forward                              / total_action_taken
        action_forward_pub.publish(msg)

        msg.data = forward_with_turn_little_right       / total_action_taken
        action_forward_with_turn_little_right_pub.publish(msg)

        msg.data = forward_with_turn_hard_right       / total_action_taken
        action_forward_with_turn_hard_right_pub.publish(msg)

        msg.data = forward_with_turn_veryhard_right       / total_action_taken
        action_forward_with_turn_veryhard_right_pub.publish(msg)

def init_env(episode, total_episode):
    threshold = math.ceil(total_episode / 3)
    print("threshold", threshold)
    if episode <= threshold:
        world_x = 0.1
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1

    # if episode % 3 == 1:
    elif (threshold < episode < (2 * threshold)):
        world_x = 1.60
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 0.0

    # if episode % 3 == 2:
    elif ((2 * threshold) <= episode <= total_episode):
        world_x = 1.5
        world_y = -1.0
        world_orientation_x = 0
        world_orientation_y = 0
        world_orientation_z = 3.14
        world_orientation_w = 0

    else:
        world_x = 0.1
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1

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
        resp = gazebo_set_state_srv( state_msg )
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
    
    end = False #to start the new episode
    return end

def get_state(front, rightfront, right):
    global state
    end = False                                         

    if (front=="veryFar" and rightfront=="far" and right=="veryFar"):
        state = 0                                                                                   ## 
    elif (front=="veryFar" and rightfront=="far" and right=="far"):
        state = 1                                      
    elif (front=="veryFar" and rightfront=="far" and right=="medium"):
        state = 2                                       
    elif (front=="veryFar" and rightfront=="far" and right=="close"):
        state = 3                                       
    elif (front=="veryFar" and rightfront=="far" and right=="veryClose"):               
        state = 4                                                                                   ##

    elif (front=="veryFar" and rightfront=="medium" and right=="veryFar"):
        state = 5                                                                                   ## 
    elif (front=="veryFar" and rightfront=="medium" and right=="far"):
        state = 6                                      
    elif (front=="veryFar" and rightfront=="medium" and right=="medium"):
        state = 7                                                                                       #FORWARD!
    elif (front=="veryFar" and rightfront=="medium" and right=="close"):
        state = 8                                       
    elif (front=="veryFar" and rightfront=="medium" and right=="veryClose"):
        state = 9                                                                                   ##   

    elif (front=="veryFar" and rightfront=="close" and right=="veryFar"):
        state = 10                                                                                      ## 
    elif (front=="veryFar" and rightfront=="close" and right=="far"):
        state = 11                                      
    elif (front=="veryFar" and rightfront=="close" and right=="medium"):
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
    elif (front=="far" and rightfront=="far" and right=="medium"):
        state = 17                                       
    elif (front=="far" and rightfront=="far" and right=="close"):
        state = 18                                       
    elif (front=="far" and rightfront=="far" and right=="veryClose"):
        state = 19  

    elif (front=="far" and rightfront=="medium" and right=="veryFar"):
        state = 20                                                                                          ## 
    elif (front=="far" and rightfront=="medium" and right=="far"):
        state = 21                                      
    elif (front=="far" and rightfront=="medium" and right=="medium"):
        state = 22                                                                                  #FORWARD!
    elif (front=="far" and rightfront=="medium" and right=="close"):
        state = 23                                       
    elif (front=="far" and rightfront=="medium" and right=="veryClose"):
        state = 24                                                                                  ## 

    elif (front=="far" and rightfront=="close" and right=="veryFar"):
        state = 25                                                                                      ## 
    elif (front=="far" and rightfront=="close" and right=="far"):
        state = 26                                      
    elif (front=="far" and rightfront=="close" and right=="medium"):
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
    elif (front=="medium" and rightfront=="far" and right=="medium"):
        state = 32                                       
    elif (front=="medium" and rightfront=="far" and right=="close"):
        state = 33                                       
    elif (front=="medium" and rightfront=="far" and right=="veryClose"):
        state = 34                                                                                          ## 

    elif (front=="medium" and rightfront=="medium" and right=="veryFar"):
        state = 35                                                                                      ## 
    elif (front=="medium" and rightfront=="medium" and right=="far"):
        state = 36                                      
    elif (front=="medium" and rightfront=="medium" and right=="medium"):
        state = 37                                       
    elif (front=="medium" and rightfront=="medium" and right=="close"):
        state = 37                                       
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
        state = 70                                                                                          ##
    elif (front=="veryClose" and rightfront=="close" and right=="far"):
        state = 71                                                                                          ##
    elif (front=="veryClose" and rightfront=="close" and right=="medium"):
        state = 72                                                                                              ##
    elif (front=="veryClose" and rightfront=="close" and right=="close"):
        state = 73                                                                                              ##
    elif (front=="veryClose" and rightfront=="close" and right=="veryClose"):
        state = 74                                                                                          ##


    print("STATE ", state)

    if state==74 or state== 73 or state ==72 or state==71 or state ==70 or state==69 or state == 68 or state== 67 or state == 66 or state==65 or state ==64 or state==63 or state ==62 or state==61 or state ==60 or state== 59 or state == 55 or state ==54 or state ==50 or state ==49 or state ==45 or state ==44 or state == 40 or state ==39 or state == 35 or state ==34 or state ==30 or state ==29 or state ==25 or state ==24 or state ==20 or state ==15 or state ==14 or state ==10 or state ==9 or state ==5 or state ==4 or state ==0 :
        rospy.loginfo("Crashed the wall/Too far from the wall - Resetting Gazebo World")
        print("Dy el state el bawazet om el denya", state)
        reward = -1
        end = True
    else:
        reward = 0
 
    return state, reward, end

def get_state_initial(front, rightfront, right):
    global state

    if (front=="veryFar" and rightfront=="far" and right=="veryFar"):
        state = 0                                                                                   ## 
    elif (front=="veryFar" and rightfront=="far" and right=="far"):
        state = 1                                      
    elif (front=="veryFar" and rightfront=="far" and right=="medium"):
        state = 2                                       
    elif (front=="veryFar" and rightfront=="far" and right=="close"):
        state = 3                                       
    elif (front=="veryFar" and rightfront=="far" and right=="veryClose"):               
        state = 4                                                                                   ##

    elif (front=="veryFar" and rightfront=="medium" and right=="veryFar"):
        state = 5                                                                                   ## 
    elif (front=="veryFar" and rightfront=="medium" and right=="far"):
        state = 6                                      
    elif (front=="veryFar" and rightfront=="medium" and right=="medium"):
        state = 7                                       
    elif (front=="veryFar" and rightfront=="medium" and right=="close"):
        state = 8                                       
    elif (front=="veryFar" and rightfront=="medium" and right=="veryClose"):
        state = 9                                                                                   ##   

    elif (front=="veryFar" and rightfront=="close" and right=="veryFar"):
        state = 10                                                                                      ## 
    elif (front=="veryFar" and rightfront=="close" and right=="far"):
        state = 11                                      
    elif (front=="veryFar" and rightfront=="close" and right=="medium"):
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
    elif (front=="far" and rightfront=="far" and right=="medium"):
        state = 17                                       
    elif (front=="far" and rightfront=="far" and right=="close"):
        state = 18                                       
    elif (front=="far" and rightfront=="far" and right=="veryClose"):
        state = 19  

    elif (front=="far" and rightfront=="medium" and right=="veryFar"):
        state = 20                                                                                          ## 
    elif (front=="far" and rightfront=="medium" and right=="far"):
        state = 21                                      
    elif (front=="far" and rightfront=="medium" and right=="medium"):
        state = 22                                       
    elif (front=="far" and rightfront=="medium" and right=="close"):
        state = 23                                       
    elif (front=="far" and rightfront=="medium" and right=="veryClose"):
        state = 24                                                                                  ## 

    elif (front=="far" and rightfront=="close" and right=="veryFar"):
        state = 25                                                                                      ## 
    elif (front=="far" and rightfront=="close" and right=="far"):
        state = 26                                      
    elif (front=="far" and rightfront=="close" and right=="medium"):
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
    elif (front=="medium" and rightfront=="far" and right=="medium"):
        state = 32                                       
    elif (front=="medium" and rightfront=="far" and right=="close"):
        state = 33                                       
    elif (front=="medium" and rightfront=="far" and right=="veryClose"):
        state = 34                                                                                          ## 

    elif (front=="medium" and rightfront=="medium" and right=="veryFar"):
        state = 35                                                                                      ## 
    elif (front=="medium" and rightfront=="medium" and right=="far"):
        state = 36                                      
    elif (front=="medium" and rightfront=="medium" and right=="medium"):
        state = 37                                       
    elif (front=="medium" and rightfront=="medium" and right=="close"):
        state = 37                                       
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
        state = 70                                                                                          ##
    elif (front=="veryClose" and rightfront=="close" and right=="far"):
        state = 71                                                                                          ##
    elif (front=="veryClose" and rightfront=="close" and right=="medium"):
        state = 72                                                                                              ##
    elif (front=="veryClose" and rightfront=="close" and right=="close"):
        state = 73                                                                                              ##
    elif (front=="veryClose" and rightfront=="close" and right=="veryClose"):
        state = 74                                                                                          ##


    return state

def Qlearn():
    global left
    global front
    global right
    global rightfront
    global total_training_rewards
    global total_action_taken
    global episode
    global epsilon
    global state
    global f
    global x
    global count
    f=0
    global steps
    steps=0

    q_table = build_q_table()
    episode = 0
    while episode < (MAX_EPISODE/3):
        if rospy.is_shutdown():
            break
        end = init_env(episode, MAX_EPISODE)
        reward = 0
        total_training_rewards = 0
        state = get_state_initial(front, rightfront, right)  
        steps=0
        while not end and not rospy.is_shutdown() and steps<=30:            # steps 30 straight wall  # were 20 for left  #were 45 u turn
            steps+=1
            print("step: ", steps)
            act = actionDecision(state, q_table)

            move_robot_by_action(act)
            time.sleep(1)
            
            if x>epsilon and ((state==7 and act == 'FORWARD') or (state==22 and act=='FORWARD')):
                f+=1
                total_action_taken += 1
            elif x>epsilon and ((state==7 and act != 'FORWARD') or (state==22 and act!='FORWARD')):
                total_action_taken += 1           
            
            next_state, reward, end = get_state(front, rightfront, right)

            print("the value of x is: ", x)    
            print("the value of epsilon is: ", epsilon)

            q_predict = q_table.loc[state, act]
            q_target = reward + GAMMA * q_table.iloc[next_state].max()

            q_table.loc[state, act] += ALPHA * (q_target - q_predict)

            state = next_state
            total_training_rewards += reward
     
        if steps>10:
            epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay*(episode))
        print("epsilon ", epsilon)
        #Adding the total reward
        training_rewards.append(total_training_rewards)
        
        if total_action_taken != 0:
            ratio_forward_action_taken.append(f/total_action_taken)
            ratio_forward_lil_left_action_taken.append(forward_with_turn_little_left/total_action_taken)
            ratio_forward_lil_right_action_taken.append(forward_with_turn_little_right/total_action_taken)

            epsilons.append(epsilon)

        episode = episode +1
        publish_all()
        save_to_filename = "%s/Q_Table_episode_%d.csv" % (rospack_path, episode)
        q_table.to_csv(save_to_filename)

    print (q_table)
    #xx = range(episode)
    plt.figure(figsize=(10,6))

    plt.plot(range(len(ratio_forward_action_taken)), ratio_forward_action_taken)
    plt.xlabel('Episode')
    plt.ylabel('Ratio Forward action taken')
    plt.title('Ratio Forward action taken in all episodes') 
    plt.show()    
    return q_table

def laserscan_callback(msg):
    global reward
    global prior_state_
    global state

    global range_front
    global range_right
    global range_left
    global range_front_right

    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left, min_front_right, i_front_right
    
    global front
    global right
    global left
    global rightfront

    ranges = msg.ranges
    
    # get the range FOV (Field of View)
    range_front[:15] = msg.ranges[15:0:-1]  # Front1 FOV (between 5 to -5 degrees)
    range_front[15:] = msg.ranges[-1:-15:-1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[225:290]  # right FOV (between 300 to 345 degrees)
    range_front_right= msg.ranges[291:320]
    range_left=msg.ranges[85:95]

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_front_right,i_front_right = min( (range_front_right[i_front_right],i_front_right) for i_front_right in range(len(range_front_right)) )

    min_left,i_left = min( (range_left[i_left],i_left) for i_left in range(len(range_left)) )

    # front states
    if (HIT_DISTANCE_THRESHOLD< min_front< CLOSE_THRESHOLD):
        front="veryClose"
    elif (CLOSE_THRESHOLD < min_front < NORMAL_THRESHOLD):
        front = "close"
    elif (NORMAL_THRESHOLD <= min_front < FAR_THRESHOLD):
        front = "medium"
    elif (FAR_THRESHOLD <min_front < LOST_DISTANCE_THRESHOLD):
        front = "far"
    elif (LOST_DISTANCE_THRESHOLD < min_front):
        front= "veryFar"
    

    # right states
    if (HIT_DISTANCE_THRESHOLD< min_right< CLOSE_THRESHOLD):
        right="veryClose"
    elif (CLOSE_THRESHOLD < min_right < NORMAL_THRESHOLD):
        right = "close"
    elif (NORMAL_THRESHOLD < min_right < FAR_THRESHOLD):
        right = "medium"
    elif (FAR_THRESHOLD <min_right < LOST_DISTANCE_THRESHOLD):
        right = "far"
    elif (LOST_DISTANCE_THRESHOLD < min_right):
        right= "veryFar"

    #right front states
    elif (CLOSE_THRESHOLD < min_front_right < NORMAL_THRESHOLD):
        rightfront = "close"
    elif (NORMAL_THRESHOLD < min_front_right < LOST_DISTANCE_THRESHOLD):
        rightfront = "medium"
    elif (min_front_right > LOST_DISTANCE_THRESHOLD):
        rightfront = "far"

#HIT_DISTANCE_THRESHOLD = 0.15 
#CLOSE_THRESHOLD = 0.20
#NORMAL_THRESHOLD = 0.30
#FAR_THRESHOLD = 0.40
#LOST_DISTANCE_THRESHOLD = 0.45
    

if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service('/gazebo/set_model_state')

    gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    
    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    rate = rospy.Rate(5.0)

    rospy.sleep(3.0)

    q_table = Qlearn()
    # rospy.spin()
    print("====== Q TABLE AFTER LEARNING ======")
    # print(q_table)
    print(" ")