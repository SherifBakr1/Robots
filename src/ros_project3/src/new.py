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
action_forward_pub                          = rospy.Publisher('action_forward', Float32, queue_size=1)
action_forward_with_turn_little_right_pub   = rospy.Publisher('action_forward_with_turn_little_right', Float32, queue_size=1)
action_forward_with_hard_left               = rospy.Publisher('action_forward_with_hard_left', Float32, queue_size=1)
action_forward_with_hard_right              = rospy.Publisher('action_forward_with_hard_right', Float32, queue_size=1)
episode_pub                                 = rospy.Publisher('episode', Int64, queue_size=1)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
HIT_DISTANCE_THRESHOLD = 0.25                #hit
LOST_DISTANCE_THRESHOLD =0.80              #very far

VCLOSE_THRESHOLD= 0.29
CLOSE_THRESHOLD = 0.33
NORMAL0_THRESHOLD=0.37 
NORMAL1_THRESHOLD = 0.41
NORMAL2_THRESHOLD = 0.45
NORMAL3_THRESHOLD = 0.49
NORMAL4_THRESHOLD= 0.53
NORMAL5_THRESHOLD= 0.57
NORMAL6_THRESHOLD= 0.61
NORMAL7_THRESHOLD= 0.65
NORMAL8_THRESHOLD= 0.69
FAR_THRESHOLD = 0.73
VERY_FAR_THRESHOLD = 0.77

N_STATES = 168
N_ACTIONS = 7

# Initialize all variables
range_front = []
range_right = []
range_left  = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0
front = ""
left = ""
right = ""
state = 0

ACTIONS     = ['FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT', 'FORWARD_AND_HARD_LEFT', 'FORWARD_AND_HARD_RIGHT', 'FORWARD_AND_V_HARD_RIGHT', 'FORWARD_AND_V_HARD_LEFT']
#STATES      = ['RIGHT_WALL_CLOSE', 'RIGHT_WALL_MEDIUM', 'RIGHT_WALL_FAR', 'FRONT_WALL_MEDIUM', 'RIGHT_WALL_SUDDENLY_NOT_FOUND'] 
#N_STATES    = len(STATES)
    
MAX_EPISODE = 600   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.8   # discount rate
ALPHA       = 0.2  # (1-alpha)
EPSILON     = 0.9

episode = 0
#global epsilon
#epsilon = 0.9
#max_epsilon = EPSILON
#min_epsilon = 0.001       
#decay = 0.015         

total_training_rewards = 0
training_rewards = []  
epsilons = []
total_action_taken = 0
forward_with_turn_little_left       = 0
forward                             = 0
forward_with_turn_little_right      = 0
forward_and_hard_left                       = 0
forward_and_hard_right                      = 0

ratio_forward_action_taken                  = []  
ratio_forward_lil_left_action_taken          = []  
ratio_forward_lil_right_action_taken          = []  
ratio_forward_and_hard_left                   = []  
ratio_forward_and_hard_right                   = []  
ratio_forward_and_V_hard_right                   = []  
ratio_forward_and_V_hard_left                   = []  


def build_q_table():
    global N_STATES
    global ACTIONS
    table = pd.DataFrame(
        np.zeros((N_STATES, len(ACTIONS)), dtype=float),
        columns=ACTIONS
    )
    return table

def actor(observation, q_table, epsilon):
    rand= np.random.uniform()
    local_epsilon= epsilon
    #print("The random number that decides the actions is", rand)
    #print("The value of epsilon is", local_epsilon)
    if rand >= local_epsilon:
        state_action = q_table.loc[observation, :]
        print("BEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEST")
        action = np.random.choice(state_action[state_action == np.max(state_action)].index)
    else:
        print("Random choice has been taken")
        action = np.random.choice(ACTIONS)
    return action

def move_robot(speed_linear_x, speed_angular_z):
    msg = Twist()
    msg.linear.x = speed_linear_x
    msg.angular.z = speed_angular_z
    cmd_vel_pub.publish(msg)
  
def move_robot_by_action(action):
    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':

        move_robot(0.04, 0.15)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD':

        move_robot(0.05, 0.0)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':

        move_robot(0.04, -0.15)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_HARD_LEFT':

        move_robot(0.04, 0.20)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_HARD_RIGHT':
   
        move_robot(0.04, -0.20)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_V_HARD_LEFT':

        move_robot(0.04,0.25)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)
    elif action == 'FORWARD_AND_V_HARD_RIGHT':

        move_robot(0.04, -0.25)
        rospy.sleep(1.2)
        move_robot(0.0,0.0)
        rospy.sleep(0.5)

def publish_all():
    global episode
    global total_action_taken
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global forward_and_hard_left
    global forward_and_hard_right

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
        msg.data = forward                              / total_action_taken
        action_forward_pub.publish(msg)
        msg.data = forward_with_turn_little_right       / total_action_taken
        action_forward_with_turn_little_right_pub.publish(msg)
        msg.data = forward_and_hard_left                      / total_action_taken
        action_forward_with_hard_left.publish(msg)
        msg.data = forward_and_hard_right                       / total_action_taken
        action_forward_with_hard_right.publish(msg)

def init_env(episode, total_episode):
    
    threshold = math.ceil(total_episode / 3)
    print(threshold)
    if episode <= threshold:
        world_x = 1.1
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1
        
    elif (threshold < episode < (2 * threshold)):
        world_x = 1.7
        world_y = -2.0
        #world_y = -0.9
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1

    elif ((2 * threshold) <= episode <= total_episode):
        world_x = 1.5
        world_y = -1.0
        world_orientation_x = 0
        world_orientation_y = 0
        world_orientation_z = 1
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
    
def get_state(front, right):
    global state
    end = False                                         

    if (right=="vclose" and front=="vclose"):
        state = 0                                      
    elif (right=="vclose" and front=="close"):
        state = 1                                      
    elif (right=="vclose" and front =="normal0"):
        state = 2
    elif (right=="vclose" and front =="normal1"):
        state = 3
    elif (right=="vclose" and front =="normal2"):
        state = 4
    elif (right=="vclose" and front =="normal3"):
        state = 5
    elif (right=="vclose" and front =="normal4"):
        state = 6    
    elif (right=="vclose" and front =="normal5"):
        state = 7        
    elif (right=="vclose" and front =="normal6"):
        state = 8
    elif (right=="vclose" and front =="normal7"):
        state = 9    
    elif (right=="vclose" and front =="normal8"):
        state = 10    
    elif (right=="vclose" and front =="far"):
        state = 11    
    elif (right=="vclose" and front =="vfar"):
        state = 12    
    elif (right=="close" and front =="vclose"):
        state = 13    
    elif (right=="close" and front =="close"):
        state = 14    
    elif (right=="close" and front =="normal0"):
        state = 15    

    elif (right=="close" and front =="normal1"):
        state = 16        
     
    elif (right=="close" and front =="normal2"):
        state = 17     

    elif (right=="close" and front =="normal3"):
        state = 18       

    elif (right=="close" and front =="normal4"):
        state = 19 
   
    elif (right=="close" and front =="normal5"):
        state = 20  
    
    elif (right=="close" and front =="normal6"):
        state = 21 
     
    elif (right=="close" and front =="normal7"):
        state = 22   
  
    elif (right=="close" and front =="normal8"):
        state = 23   
  
    elif (right=="close" and front =="far"):
        state = 24 
     
    elif (right=="close" and front =="vfar"):
        state = 25       
    elif (right=="normal0" and front =="vclose"):
        state = 26       
    elif (right=="normal0" and front =="close"):
        state = 27       
    elif (right=="normal0" and front =="normal0"):
        state = 28   
    elif (right=="normal0" and front =="normal1"):
        state = 29  
    elif (right=="normal0" and front =="normal2"):
        state = 30  
    elif (right=="normal0" and front =="normal3"):
        state = 31  
    elif (right=="normal0" and front =="normal4"):
        state = 32  
    elif (right=="normal0" and front =="normal5"):
        state = 33  
    elif (right=="normal0" and front =="normal6"):
        state = 34  
    elif (right=="normal0" and front =="normal7"):
        state = 35  
    elif (right=="normal0" and front =="normal8"):
        state = 36  
    elif (right=="normal0" and front =="far"):
        state = 37  
    elif (right=="normal0" and front =="vfar"):
        state = 38  
    elif (right=="normal1" and front =="vclose"):
        state = 39  
    elif (right=="normal1" and front =="close"):
        state = 40  
    elif (right=="normal1" and front =="normal0"):
        state = 41  
    elif (right=="normal1" and front =="normal1"):
        state = 42  
    elif (right=="normal1" and front =="normal2"):
        state = 43  
    elif (right=="normal1" and front =="normal3"):
        state = 44  
    elif (right=="normal1" and front =="normal4"):
        state = 45  
    elif (right=="normal1" and front =="normal5"):
        state = 46  
    elif (right=="normal1" and front =="normal6"):
        state = 47  
    elif (right=="normal1" and front =="normal7"):
        state = 48  
    elif (right=="normal1" and front =="normal8"):
        state = 49  
    elif (right=="normal1" and front =="far"):
        state = 50 
    elif (right=="normal1" and front =="vfar"):
        state = 51  
    elif (right=="normal2" and front =="vclose"):
        state = 52  
    elif (right=="normal2" and front =="close"):
        state = 53  
    elif (right=="normal2" and front =="normal0"):
        state = 54  
    elif (right=="normal2" and front =="normal1"):
        state = 55   
    elif (right=="normal2" and front =="normal2"):
        state = 56  
    elif (right=="normal2" and front =="normal3"):
        state = 57  
    elif (right=="normal2" and front =="normal4"):
        state = 58  
    elif (right=="normal2" and front =="normal5"):
        state = 59  
    elif (right=="normal2" and front =="normal6"):
        state = 60  
    elif (right=="normal2" and front =="normal7"):
        state = 61  
    elif (right=="normal2" and front =="normal8"):
        state = 62  
    elif (right=="normal2" and front =="far"):
        state = 63  
    elif (right=="normal2" and front =="vfar"):
        state = 64  
    elif (right=="normal3" and front =="vclose"):
        state = 65  
    elif (right=="normal3" and front =="close"):
        state = 66  
    elif (right=="normal3" and front =="normal0"):
        state = 67  
    elif (right=="normal3" and front =="normal1"):
        state = 68  
    elif (right=="normal3" and front =="normal2"):
        state = 69  
    elif (right=="normal3" and front =="normal3"):
        state = 70  
    elif (right=="normal3" and front =="normal4"):
        state = 71  
    elif (right=="normal3" and front =="normal5"):
        state = 72  
    elif (right=="normal3" and front =="normal6"):
        state = 73  
    elif (right=="normal3" and front =="normal7"):
        state = 74  
    elif (right=="normal3" and front =="normal8"):
        state = 75  
    elif (right=="normal3" and front =="far"):
        state = 76  
    elif (right=="normal3" and front =="vfar"):
        state = 77  
    elif (right=="normal4" and front =="vclose"):
        state = 78 
    elif (right=="normal4" and front =="close"):
        state = 79 
    elif (right=="normal4" and front =="normal0"):
        state = 80 
    elif (right=="normal4" and front =="normal1"):
        state = 80 
    elif (right=="normal4" and front =="normal2"):
        state = 81 
    elif (right=="normal4" and front =="normal3"):
        state = 82 
    elif (right=="normal4" and front =="normal4"):
        state = 83 
    elif (right=="normal4" and front =="normal5"):
        state = 84 
    elif (right=="normal4" and front =="normal6"):
        state = 85 
    elif (right=="normal4" and front =="normal7"):
        state = 86 
    elif (right=="normal4" and front =="normal8"):
        state = 87 
    elif (right=="normal4" and front =="far"):
        state = 88 
    elif (right=="normal4" and front =="vfar"):
        state = 89 
    elif (right=="normal5" and front =="vclose"):
        state = 90 
    elif (right=="normal5" and front =="close"):
        state = 91 
    elif (right=="normal5" and front =="normal0"):
        state = 92 
    elif (right=="normal5" and front =="normal1"):
        state = 93 
    elif (right=="normal5" and front =="normal2"):
        state = 94 
    elif (right=="normal5" and front =="normal3"):
        state = 95 
    elif (right=="normal5" and front =="normal4"):
        state = 96 
    elif (right=="normal5" and front =="normal5"):
        state = 97 
    elif (right=="normal5" and front =="normal6"):
        state = 98 
    elif (right=="normal5" and front =="normal7"):
        state = 99 
    elif (right=="normal5" and front =="normal8"):
        state = 100 
    elif (right=="normal5" and front =="far"):
        state = 101 
    elif (right=="normal5" and front =="vfar"):
        state = 102 
    elif (right=="normal6" and front =="vclose"):
        state = 103 
    elif (right=="normal6" and front =="close"):
        state = 104 
    elif (right=="normal6" and front =="normal0"):
        state = 105 
    elif (right=="normal6" and front =="normal1"):
        state = 106 
    elif (right=="normal6" and front =="normal2"):
        state = 107 
    elif (right=="normal6" and front =="normal3"):
        state = 108 
    elif (right=="normal6" and front =="normal4"):
        state = 109 
    elif (right=="normal6" and front =="normal5"):
        state = 110 
    elif (right=="normal6" and front =="normal6"):
        state = 111 
    elif (right=="normal6" and front =="normal7"):
        state = 112 
    elif (right=="normal6" and front =="normal8"):
        state = 113 
    elif (right=="normal6" and front =="far"):
        state = 114 
    elif (right=="normal6" and front =="vfar"):
        state = 115 
    elif (right=="normal7" and front =="vclose"):
        state = 116 
    elif (right=="normal7" and front =="close"):
        state = 117 
    elif (right=="normal7" and front =="normal0"):
        state = 118 
    elif (right=="normal7" and front =="normal1"):
        state = 119 
    elif (right=="normal7" and front =="normal2"):
        state = 120 
    elif (right=="normal7" and front =="normal3"):
        state = 121 
    elif (right=="normal7" and front =="normal4"):
        state = 122 
    elif (right=="normal7" and front =="normal5"):
        state = 123 
    elif (right=="normal7" and front =="normal6"):
        state = 124 
    elif (right=="normal7" and front =="normal7"):
        state = 125 
    elif (right=="normal7" and front =="normal8"):
        state = 126 
    elif (right=="normal7" and front =="far"):
        state = 127 
    elif (right=="normal7" and front =="vfar"):
        state = 128 
    elif (right=="normal8" and front =="vclose"):
        state = 129 
    elif (right=="normal8" and front =="close"):
        state = 130 
    elif (right=="normal8" and front =="normal0"):
        state = 131 
    elif (right=="normal8" and front =="normal1"):
        state = 132 
    elif (right=="normal8" and front =="normal2"):
        state = 133 
    elif (right=="normal8" and front =="normal3"):
        state = 134 
    elif (right=="normal8" and front =="normal4"):
        state = 135 
    elif (right=="normal8" and front =="normal5"):
        state = 136 
    elif (right=="normal8" and front =="normal6"):
        state = 137 
    elif (right=="normal8" and front =="normal7"):
        state = 138 
    elif (right=="normal8" and front =="normal8"):
        state = 139 
    elif (right=="normal8" and front =="far"):
        state = 140 
    elif (right=="normal8" and front =="vfar"):
        state = 141 
    elif (right=="far" and front =="vclose"):
        state = 142 
    elif (right=="far" and front =="close"):
        state = 143 
    elif (right=="far" and front =="normal0"):
        state = 144 
    elif (right=="far" and front =="normal1"):
        state = 145 
    elif (right=="far" and front =="normal2"):
        state = 146 
    elif (right=="far" and front =="normal3"):
        state = 147 
    elif (right=="far" and front =="normal4"):
        state = 148 
    elif (right=="far" and front =="normal5"):
        state = 149 
    elif (right=="far" and front =="normal6"):
        state = 150 
    elif (right=="far" and front =="normal7"):
        state = 151 
    elif (right=="far" and front =="normal8"):
        state = 152 
    elif (right=="far" and front =="far"):
        state = 153 
    elif (right=="far" and front =="vfar"):
        state = 154 
    elif (right=="vfar" and front =="vclose"):
        state = 155 
    elif (right=="vfar" and front =="close"):
        state = 156 
    elif (right=="vfar" and front =="normal0"):
        state = 157 
    elif (right=="vfar" and front =="normal1"):
        state = 158 
    elif (right=="vfar" and front =="normal2"):
        state = 159 
    elif (right=="vfar" and front =="normal3"):
        state = 160 
    elif (right=="vfar" and front =="normal4"):
        state = 161 
    elif (right=="vfar" and front =="normal5"):
        state = 162 
    elif (right=="vfar" and front =="normal6"):
        state = 163 
    elif (right=="vfar" and front =="normal7"):
        state = 164 
    elif (right=="vfar" and front =="normal8"):
        state = 165 
    elif (right=="vfar" and front =="far"):
        state = 166 
    elif (right=="vfar" and front =="vfar"):
        state = 167

    print("state: ", state)

    if min_front <= HIT_DISTANCE_THRESHOLD or  min_right <= HIT_DISTANCE_THRESHOLD or min_right> LOST_DISTANCE_THRESHOLD:
        print("HIT_DISTANCE_THRESHOLD", HIT_DISTANCE_THRESHOLD)
        rospy.loginfo("Crashed the wall/Too far - Resetting Gazebo World")
        reward = -10
        end = True
    else:
        reward = 0
 
    return state, reward, end

def get_state_initial(front, right):
    global state                                       

    if (right=="vclose" and front=="vclose"):
        state = 0                                      
    elif (right=="vclose" and front=="close"):
        state = 1                                      
    elif (right=="vclose" and front =="normal0"):
        state = 2
    elif (right=="vclose" and front =="normal1"):
        state = 3
    elif (right=="vclose" and front =="normal2"):
        state = 4
    elif (right=="vclose" and front =="normal3"):
        state = 5
    elif (right=="vclose" and front =="normal4"):
        state = 6    
    elif (right=="vclose" and front =="normal5"):
        state = 7        
    elif (right=="vclose" and front =="normal6"):
        state = 8
    elif (right=="vclose" and front =="normal7"):
        state = 9    
    elif (right=="vclose" and front =="normal8"):
        state = 10    
    elif (right=="vclose" and front =="far"):
        state = 11    
    elif (right=="vclose" and front =="vfar"):
        state = 12    
    elif (right=="close" and front =="vclose"):
        state = 13    
    elif (right=="close" and front =="close"):
        state = 14    
    elif (right=="close" and front =="normal0"):
        state = 15    
    elif (right=="close" and front =="normal1"):
        state = 16              
    elif (right=="close" and front =="normal2"):
        state = 17       
    elif (right=="close" and front =="normal3"):
        state = 18       
    elif (right=="close" and front =="normal4"):
        state = 19       
    elif (right=="close" and front =="normal5"):
        state = 20       
    elif (right=="close" and front =="normal6"):
        state = 21       
    elif (right=="close" and front =="normal7"):
        state = 22       
    elif (right=="close" and front =="normal8"):
        state = 23       
    elif (right=="close" and front =="far"):
        state = 24       
    elif (right=="close" and front =="vfar"):
        state = 25       
    elif (right=="normal0" and front =="vclose"):
        state = 26       
    elif (right=="normal0" and front =="close"):
        state = 27       
    elif (right=="normal0" and front =="normal0"):
        state = 28   
    elif (right=="normal0" and front =="normal1"):
        state = 29  
    elif (right=="normal0" and front =="normal2"):
        state = 30  
    elif (right=="normal0" and front =="normal3"):
        state = 31  
    elif (right=="normal0" and front =="normal4"):
        state = 32  
    elif (right=="normal0" and front =="normal5"):
        state = 33  
    elif (right=="normal0" and front =="normal6"):
        state = 34  
    elif (right=="normal0" and front =="normal7"):
        state = 35  
    elif (right=="normal0" and front =="normal8"):
        state = 36  
    elif (right=="normal0" and front =="far"):
        state = 37  
    elif (right=="normal0" and front =="vfar"):
        state = 38  
    elif (right=="normal1" and front =="vclose"):
        state = 39  
    elif (right=="normal1" and front =="close"):
        state = 40  
    elif (right=="normal1" and front =="normal0"):
        state = 41  
    elif (right=="normal1" and front =="normal1"):
        state = 42  
    elif (right=="normal1" and front =="normal2"):
        state = 43  
    elif (right=="normal1" and front =="normal3"):
        state = 44  
    elif (right=="normal1" and front =="normal4"):
        state = 45  
    elif (right=="normal1" and front =="normal5"):
        state = 46  
    elif (right=="normal1" and front =="normal6"):
        state = 47  
    elif (right=="normal1" and front =="normal7"):
        state = 48  
    elif (right=="normal1" and front =="normal8"):
        state = 49  
    elif (right=="normal1" and front =="far"):
        state = 50 
    elif (right=="normal1" and front =="vfar"):
        state = 51  
    elif (right=="normal2" and front =="vclose"):
        state = 52  
    elif (right=="normal2" and front =="close"):
        state = 53  
    elif (right=="normal2" and front =="normal0"):
        state = 54  
    elif (right=="normal2" and front =="normal1"):
        state = 55   
    elif (right=="normal2" and front =="normal2"):
        state = 56  
    elif (right=="normal2" and front =="normal3"):
        state = 57  
    elif (right=="normal2" and front =="normal4"):
        state = 58  
    elif (right=="normal2" and front =="normal5"):
        state = 59  
    elif (right=="normal2" and front =="normal6"):
        state = 60  
    elif (right=="normal2" and front =="normal7"):
        state = 61  
    elif (right=="normal2" and front =="normal8"):
        state = 62  
    elif (right=="normal2" and front =="far"):
        state = 63  
    elif (right=="normal2" and front =="vfar"):
        state = 64  
    elif (right=="normal3" and front =="vclose"):
        state = 65  
    elif (right=="normal3" and front =="close"):
        state = 66  
    elif (right=="normal3" and front =="normal0"):
        state = 67  
    elif (right=="normal3" and front =="normal1"):
        state = 68  
    elif (right=="normal3" and front =="normal2"):
        state = 69  
    elif (right=="normal3" and front =="normal3"):
        state = 70  
    elif (right=="normal3" and front =="normal4"):
        state = 71  
    elif (right=="normal3" and front =="normal5"):
        state = 72  
    elif (right=="normal3" and front =="normal6"):
        state = 73  
    elif (right=="normal3" and front =="normal7"):
        state = 74  
    elif (right=="normal3" and front =="normal8"):
        state = 75  
    elif (right=="normal3" and front =="far"):
        state = 76  
    elif (right=="normal3" and front =="vfar"):
        state = 77  
    elif (right=="normal4" and front =="vclose"):
        state = 78 
    elif (right=="normal4" and front =="close"):
        state = 79 
    elif (right=="normal4" and front =="normal0"):
        state = 80 
    elif (right=="normal4" and front =="normal1"):
        state = 80 
    elif (right=="normal4" and front =="normal2"):
        state = 81 
    elif (right=="normal4" and front =="normal3"):
        state = 82 
    elif (right=="normal4" and front =="normal4"):
        state = 83 
    elif (right=="normal4" and front =="normal5"):
        state = 84 
    elif (right=="normal4" and front =="normal6"):
        state = 85 
    elif (right=="normal4" and front =="normal7"):
        state = 86 
    elif (right=="normal4" and front =="normal8"):
        state = 87 
    elif (right=="normal4" and front =="far"):
        state = 88 
    elif (right=="normal4" and front =="vfar"):
        state = 89 
    elif (right=="normal5" and front =="vclose"):
        state = 90 
    elif (right=="normal5" and front =="close"):
        state = 91 
    elif (right=="normal5" and front =="normal0"):
        state = 92 
    elif (right=="normal5" and front =="normal1"):
        state = 93 
    elif (right=="normal5" and front =="normal2"):
        state = 94 
    elif (right=="normal5" and front =="normal3"):
        state = 95 
    elif (right=="normal5" and front =="normal4"):
        state = 96 
    elif (right=="normal5" and front =="normal5"):
        state = 97 
    elif (right=="normal5" and front =="normal6"):
        state = 98 
    elif (right=="normal5" and front =="normal7"):
        state = 99 
    elif (right=="normal5" and front =="normal8"):
        state = 100 
    elif (right=="normal5" and front =="far"):
        state = 101 
    elif (right=="normal5" and front =="vfar"):
        state = 102 
    elif (right=="normal6" and front =="vclose"):
        state = 103 
    elif (right=="normal6" and front =="close"):
        state = 104 
    elif (right=="normal6" and front =="normal0"):
        state = 105 
    elif (right=="normal6" and front =="normal1"):
        state = 106 
    elif (right=="normal6" and front =="normal2"):
        state = 107 
    elif (right=="normal6" and front =="normal3"):
        state = 108 
    elif (right=="normal6" and front =="normal4"):
        state = 109 
    elif (right=="normal6" and front =="normal5"):
        state = 110 
    elif (right=="normal6" and front =="normal6"):
        state = 111 
    elif (right=="normal6" and front =="normal7"):
        state = 112 
    elif (right=="normal6" and front =="normal8"):
        state = 113 
    elif (right=="normal6" and front =="far"):
        state = 114 
    elif (right=="normal6" and front =="vfar"):
        state = 115 
    elif (right=="normal7" and front =="vclose"):
        state = 116 
    elif (right=="normal7" and front =="close"):
        state = 117 
    elif (right=="normal7" and front =="normal0"):
        state = 118 
    elif (right=="normal7" and front =="normal1"):
        state = 119 
    elif (right=="normal7" and front =="normal2"):
        state = 120 
    elif (right=="normal7" and front =="normal3"):
        state = 121 
    elif (right=="normal7" and front =="normal4"):
        state = 122 
    elif (right=="normal7" and front =="normal5"):
        state = 123 
    elif (right=="normal7" and front =="normal6"):
        state = 124 
    elif (right=="normal7" and front =="normal7"):
        state = 125 
    elif (right=="normal7" and front =="normal8"):
        state = 126 
    elif (right=="normal7" and front =="far"):
        state = 127 
    elif (right=="normal7" and front =="vfar"):
        state = 128 
    elif (right=="normal8" and front =="vclose"):
        state = 129 
    elif (right=="normal8" and front =="close"):
        state = 130 
    elif (right=="normal8" and front =="normal0"):
        state = 131 
    elif (right=="normal8" and front =="normal1"):
        state = 132 
    elif (right=="normal8" and front =="normal2"):
        state = 133 
    elif (right=="normal8" and front =="normal3"):
        state = 134 
    elif (right=="normal8" and front =="normal4"):
        state = 135 
    elif (right=="normal8" and front =="normal5"):
        state = 136 
    elif (right=="normal8" and front =="normal6"):
        state = 137 
    elif (right=="normal8" and front =="normal7"):
        state = 138 
    elif (right=="normal8" and front =="normal8"):
        state = 139 
    elif (right=="normal8" and front =="far"):
        state = 140 
    elif (right=="normal8" and front =="vfar"):
        state = 141 
    elif (right=="far" and front =="vclose"):
        state = 142 
    elif (right=="far" and front =="close"):
        state = 143 
    elif (right=="far" and front =="normal0"):
        state = 144 
    elif (right=="far" and front =="normal1"):
        state = 145 
    elif (right=="far" and front =="normal2"):
        state = 146 
    elif (right=="far" and front =="normal3"):
        state = 147 
    elif (right=="far" and front =="normal4"):
        state = 148 
    elif (right=="far" and front =="normal5"):
        state = 149 
    elif (right=="far" and front =="normal6"):
        state = 150 
    elif (right=="far" and front =="normal7"):
        state = 151 
    elif (right=="far" and front =="normal8"):
        state = 152 
    elif (right=="far" and front =="far"):
        state = 153 
    elif (right=="far" and front =="vfar"):
        state = 154 
    elif (right=="vfar" and front =="vclose"):
        state = 155 
    elif (right=="vfar" and front =="close"):
        state = 156 
    elif (right=="vfar" and front =="normal0"):
        state = 157 
    elif (right=="vfar" and front =="normal1"):
        state = 158 
    elif (right=="vfar" and front =="normal2"):
        state = 159 
    elif (right=="vfar" and front =="normal3"):
        state = 160 
    elif (right=="vfar" and front =="normal4"):
        state = 161 
    elif (right=="vfar" and front =="normal5"):
        state = 162 
    elif (right=="vfar" and front =="normal6"):
        state = 163 
    elif (right=="vfar" and front =="normal7"):
        state = 164 
    elif (right=="vfar" and front =="normal8"):
        state = 165 
    elif (right=="vfar" and front =="far"):
        state = 166 
    elif (right=="vfar" and front =="vfar"):
        state = 167
                                                    
    return state

### Qlearning algoritm
def Qlearn():
    global steps
    steps=0
    global left
    global front
    global right
    global total_training_rewards
    global total_action_taken
    global episode
    global state
    global f
    f=0
    global l
    l=0
    q_table = build_q_table()
    episode = 201
    epsilon = 0.9

    plt.figure(figsize=(10,6))
    plt.ion()
    plt.show()
    while episode < (MAX_EPISODE*2/3):
        if rospy.is_shutdown():
            break
        
        end = init_env(episode, MAX_EPISODE)
        #time.sleep(1)

        reward = 0
        total_training_rewards = 0
        
        state = get_state_initial(front, right)     
        print("THE VERY FIRST STATE IS", state) 
        steps=0                
        while not end and not rospy.is_shutdown() and steps <20:

            print("STEPS", steps)
            steps+=1
            print("EPSILON ISSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS", epsilon)
            act = actor(state, q_table, epsilon)

            move_robot_by_action(act)
            
            #print("One action has been taken")
            # rospy.sleep(0.1)

            next_state, reward, end = get_state(front, right)
            
            if (state == 100 or state == 99 or state == 103 or state==155 or state == 64 or state ==29 or state == 78 or state == 89 or state == 28 or state ==102 or state == 66 or state ==141 or state == 55 or state == 77 or state == 56 or state == 156 or state== 90 or state ==30 ):
                total_action_taken += 1
                if act == 'FORWARD_AND_V_HARD_LEFT' or act == 'FORWARD_AND_HARD_LEFT':
                    reward+=10
                    l+=1
                    print("Reward given")
                
            q_predict = q_table.loc[state, act]

            ### Qlearning algoritm
          
            q_target = reward + GAMMA * q_table.iloc[next_state].max()

            q_table.loc[state, act] += ALPHA * (q_target - q_predict)
            state = next_state

            total_training_rewards += reward

        epsilon = 0.1 + (0.9 - 0.1)*np.exp(-0.001*(episode-200))

        #Adding the total reward
        training_rewards.append(total_training_rewards)
        
        if total_action_taken != 0:
            #ratio_forward_action_taken.append(f/total_action_taken)
            #ratio_forward_lil_left_action_taken.append(forward_with_turn_little_left/total_action_taken)
            #ratio_forward_lil_right_action_taken.append(forward_with_turn_little_right/total_action_taken)
            #ratio_forward_and_hard_left.append(l/total_action_taken)
            ratio_forward_and_V_hard_left.append(l/total_action_taken)
 
        
            epsilons.append(epsilon)

        episode = episode +1
        print("The episode we're currently in is episode: ", episode)
        publish_all()
        save_to_filename = "%s/Q_Table_episode_%d.csv" % (rospack_path, episode)
        q_table.to_csv(save_to_filename)
        # rospy.sleep(0.1)
        
        plt.clf()
        plt.plot(range(len(ratio_forward_and_V_hard_left)), ratio_forward_and_V_hard_left, label='Ratio Forward and Hard Left')

        plt.xlabel('Episode')
        plt.ylabel('Ratio Forward and Hard Left action taken')
        plt.title('Ratio Forward and Hard Left action taken during training')
        plt.legend()
        plt.pause(0.01)

    plt.ioff()
    plt.show()    

    return q_table

def laserscan_callback(msg):
    #global reward
    #global prior_state_
    #global state

    global range_front
    global range_right
    global range_left
    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left
    
    global front
    global right
    global left
    ranges = msg.ranges
    
    # get the range FOV (Field of View)
    #range_front[:5] = msg.ranges[5:0:-1]  # Front1 FOV (between 5 to -5 degrees)
    range_front = msg.ranges[0:1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[275:276]  # right FOV (between 300 to 345 degrees)
    range_left=msg.ranges[85:95]

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_left,i_left = min( (range_left[i_left],i_left) for i_left in range(len(range_left)) )

    # front states
    if (min_front < VCLOSE_THRESHOLD):
        front = "vclose"
    elif (VCLOSE_THRESHOLD <min_front < CLOSE_THRESHOLD):
        front = "close"
    elif (CLOSE_THRESHOLD <= min_front < NORMAL0_THRESHOLD):
        front = "normal0"
    elif (NORMAL0_THRESHOLD <= min_front < NORMAL1_THRESHOLD):
        front = "normal1"
    elif (NORMAL1_THRESHOLD <= min_front < NORMAL2_THRESHOLD):
        front = "normal2"
    elif (NORMAL2_THRESHOLD <= min_front < NORMAL3_THRESHOLD):
        front = "normal3"
    elif (NORMAL3_THRESHOLD <= min_front < NORMAL4_THRESHOLD):
        front = "normal4"
    elif (NORMAL4_THRESHOLD <= min_front < NORMAL5_THRESHOLD):
        front = "normal5"
    elif (NORMAL5_THRESHOLD <= min_front < NORMAL6_THRESHOLD):
        front = "normal6"
    elif (NORMAL6_THRESHOLD <= min_front < NORMAL7_THRESHOLD):
        front = "normal7"
    elif (NORMAL7_THRESHOLD <= min_front < NORMAL8_THRESHOLD):
        front = "normal8"
    elif (NORMAL8_THRESHOLD <= min_front < FAR_THRESHOLD):
        front = "far"
    elif (min_front >= FAR_THRESHOLD):
        front = "vfar"

    # right states
    if (min_right < VCLOSE_THRESHOLD):
        right = "vclose"
    elif (VCLOSE_THRESHOLD <min_right < CLOSE_THRESHOLD):
        right = "close"
    elif (CLOSE_THRESHOLD <= min_right < NORMAL0_THRESHOLD):
        right = "normal0"
    elif (NORMAL0_THRESHOLD <= min_right < NORMAL1_THRESHOLD):
        right = "normal1"
    elif (NORMAL1_THRESHOLD <= min_right < NORMAL2_THRESHOLD):
        right = "normal2"
    elif (NORMAL2_THRESHOLD <= min_right < NORMAL3_THRESHOLD):
        right = "normal3"
    elif (NORMAL3_THRESHOLD <= min_right < NORMAL4_THRESHOLD):
        right = "normal4"
    elif (NORMAL4_THRESHOLD <= min_right < NORMAL5_THRESHOLD):
        right = "normal5"
    elif (NORMAL5_THRESHOLD <= min_right < NORMAL6_THRESHOLD):
        right = "normal6"
    elif (NORMAL6_THRESHOLD <= min_right < NORMAL7_THRESHOLD):
        right = "normal7"
    elif (NORMAL7_THRESHOLD <= min_right < NORMAL8_THRESHOLD):
        right = "normal8"
    elif (NORMAL8_THRESHOLD <= min_right < FAR_THRESHOLD):
        right = "far"
    elif (min_right >= FAR_THRESHOLD):
        right = "vfar"

if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service('/gazebo/set_model_state')

    gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    
    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    #rate = rospy.Rate(5.0)

    #rospy.sleep(3.0)

    q_table = Qlearn()
    # rospy.spin()
    print("====== Q TABLE AFTER LEARNING ======")
    # print(q_table)
    print(" ")
