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
total_action_taken2_pub                     = rospy.Publisher('total_action_taken2', Int64, queue_size=1)
action_forward_with_turn_little_left_pub    = rospy.Publisher('action_forward_with_turn_little_left', Float32, queue_size=1)
action_forward_pub                          = rospy.Publisher('action_forward', Float32, queue_size=1)
action_forward_with_turn_little_right_pub   = rospy.Publisher('action_forward_with_turn_little_right', Float32, queue_size=1)
action_forward_with_hard_left               = rospy.Publisher('action_forward_with_hard_left', Float32, queue_size=1)
action_forward_with_hard_right              = rospy.Publisher('action_forward_with_hard_right', Float32, queue_size=1)
episode_pub                                 = rospy.Publisher('episode', Int64, queue_size=1)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)      

N_STATES = 15
N_ACTIONS = 5

# Initialize all variables
range_front = []
range_right = []
range_left  = []
range_corner = []
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

ACTIONS     = ['FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT', 'FORWARD_AND_HARD_LEFT', 'FORWARD_AND_HARD_RIGHT']
    
MAX_EPISODE = 1200   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.8   # discount rate
ALPHA       = 0.1  # (1-alpha)
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
total_action_taken2 =0
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
        print("Best choice has been made")
        action = np.random.choice(state_action[state_action == np.max(state_action)].index)
    else:
        print("Random choice has been made")
        action = np.random.choice(ACTIONS)
    return action

def move_robot(speed_linear_x, speed_angular_z):
    msg = Twist()
    msg.linear.x = speed_linear_x
    msg.angular.z = speed_angular_z
    cmd_vel_pub.publish(msg)
  
def move_robot_by_action(action):
    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.16, 0.40)
        rospy.sleep(1.2)
    elif action == 'FORWARD':
        move_robot(0.16, 0.0)
        rospy.sleep(1.2)
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.16, -0.40)
        rospy.sleep(1.2)
    elif action == 'FORWARD_AND_HARD_LEFT':
        move_robot(0.10, 0.60)
        rospy.sleep(1.2)
    elif action == 'FORWARD_AND_HARD_RIGHT':
        move_robot(0.10, -0.60)
        rospy.sleep(1.2)

def stopRobot():
    move_robot(0.0,0.0)
    rospy.sleep(1)
def stopRobot2():
    move_robot(0.0,0.0)
    rospy.sleep(0.1)
def publish_all():
    global episode
    global total_action_taken
    global total_action_taken2
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

    action_msg = Int64()
    action_msg.data = total_action_taken2
    total_action_taken2_pub.publish(action_msg)

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
    
    if total_action_taken2!=0:
        #msg.data = forward_with_turn_little_left        / total_action_taken2
        #action_forward_with_turn_little_left_pub.publish(msg)
        msg.data = forward                              / total_action_taken2
        action_forward_pub.publish(msg)
        #msg.data = forward_with_turn_little_right       / total_action_taken2
        #action_forward_with_turn_little_right_pub.publish(msg)
        #msg.data = forward_and_hard_left                      / total_action_taken2
        #action_forward_with_hard_left.publish(msg)
        #msg.data = forward_and_hard_right                       / total_action_taken2
        #action_forward_with_hard_right.publish(msg)


def init_env(episode, total_episode):
    
    threshold = math.ceil(total_episode / 3)
    if episode <= threshold:
        world_x = 0.25
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1
        
    elif (threshold < episode < (2 * threshold)):
        world_x = 1.8
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1


    elif ((2 * threshold) <= episode <= total_episode):
        world_x = 1.5
        world_y = -1.0
        world_orientation_x = 0
        world_orientation_y = 0
        world_orientation_z = 3.15
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
    if (right == "vclose" and front =="close"):
        state = 0 
        reward = -100
        end = True
        return state, reward, end
    if (right == "vclose" and front =="normal"):
        state = 1
        reward = -100
        end = True
        return state, reward, end
    if (right == "vclose" and front =="far"):
        state = 2 
        reward = -100
        end = True
        return state, reward, end 
    elif (right == "close" and front == "close"):
        state = 3
        reward = -100
        end = True
        return state, reward, end
    
    elif (right == "close" and front == "normal"):
        state = 4
        #rewarded in main loop
    elif (right == "close" and front == "far"):
        state = 5
        #rewarded in main loop
    elif (right == "normal0" and front == "close"):
        state = 6
        reward = -100
        end = True
        return state, reward, end
    elif (right == "normal0" and front == "normal"):
        state = 7
        reward = 1
        end = False
        return state, reward, end
    elif (right == "normal0" and front == "far"):
        state = 8
        reward = 1
        end = False
        return state, reward, end
    elif (right == "normal1" and front == "close"):
        state = 9
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "normal1" and front == "normal"):
        state = 10
        reward = 1
        end = False
        return state, reward, end
    elif (right == "normal1" and front == "far"):
        state = 11
        reward = 1
        end = False
        return state, reward, end


    elif (right == "far" and front == "close"):
        state = 12
        reward = -100
        end = True  
        return state, reward, end  
    elif (right == "far" and front == "normal"):
        state = 13
        reward = -100
        end = True  
        return state, reward, end 
    elif (right == "far" and front == "far"):
        state = 14
        reward = -100
        end = True  
        return state, reward, end 
    
    reward= 0
    return state, reward, end

def get_state_initial(front, right):
    global state              

    if (right == "vclose" and front =="close"):
        state = 0 
    if (right == "vclose" and front =="normal"):
        state = 1
    if (right == "vclose" and front =="far"):
        state = 2 
    elif (right == "close" and front == "close"):
        state = 3
    elif (right == "close" and front == "normal"):
        state = 4
    elif (right == "close" and front == "far"):
        state = 5
    elif (right == "normal0" and front == "close"):
        state = 6
    elif (right == "normal0" and front == "normal"):
        state = 7
    elif (right == "normal0" and front == "far"):
        state = 8
    elif (right == "normal1" and front == "close"):
        state = 9
    elif (right == "normal1" and front == "normal"):
        state = 10
    elif (right == "normal1" and front == "far"):
        state = 11
    elif (right == "far" and front == "close"):
        state = 12
    elif (right == "far" and front == "normal"):
        state = 13
    elif (right == "far" and front == "far"):
        state = 14
    
    
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
    global total_action_taken2

    global episode
    global state
    global f
    f=0
    global l
    l=0
    q_table = build_q_table()
    episode = 0
    epsilon = 0.9

    plt.figure(figsize=(10,6))
    plt.ion()
    plt.show()
    while episode < (MAX_EPISODE/3):
        if rospy.is_shutdown():
            break
        
        end = init_env(episode, MAX_EPISODE)
        print("Before stopping")
        stopRobot()
        print("After stopping")

        
        total_training_rewards = 0
        
        state = get_state_initial(front, right)     
        print("THE VERY FIRST STATE IS", state) 
        steps=0                
        while not end and not rospy.is_shutdown() and steps<50:

            print("STEPS", steps)
            steps+=1
            print("EPSILON IS", epsilon)
            act = actor(state, q_table, epsilon)
            #act= 'FORWARD_AND_HARD_LEFT'
            move_robot_by_action(act)

            #stopRobot2()

            #state= get_state(front, right)

            #print("The next state is!!!!!!!!: ", state)

            #stopRobot2()


            #print("One action has been taken")
            # rospy.sleep(0.1)

            next_state, reward, end = get_state(front, right)
            
            #ADD ALL THE STATES WHERE THERE IS A WALL IN FRONT OF THE ROBOT, and it takes a hard right turn, reward it negatively
            if ((state==4) and range_corner <1.5):
                total_action_taken += 1
                if act == 'FORWARD_AND_HARD_LEFT':
                    l+=1
                    reward+=10
                if act == 'FORWARD_AND_HARD_RIGHT':
                    reward-=10

            if ((state==7 or state ==10) and range_corner <1.5):
                if act == 'FORWARD_AND_HARD_LEFT':
                    reward-=10
                if act == 'FORWARD_AND_TURN_LITTLE_LEFT':
                    reward+=10
                
            if (state==5 and range_corner<1.5):
                total_action_taken2 +=1
                if act == 'FORWARD':
                    f+=1
                    reward+=10
            
            #For u-turn
            if ((state==5 or state==8 or state==11) and (range_corner>1.5) and (act =='FORWARD_AND_HARD_LEFT' or act == 'FORWARD_AND_TURN_LITTLE_LEFT' )):
                 reward-=10

            q_predict = q_table.loc[state, act]

            ### Qlearning algoritm
          
            q_target = reward + GAMMA * q_table.iloc[next_state].max()

            q_table.loc[state, act] += ALPHA * (q_target - q_predict)
            state = next_state

            total_training_rewards += reward

        epsilon = 0.01 + (0.99 - 0.01)*np.exp(-0.01*(episode))
        #training_rewards.append(total_training_rewards)
        
        if total_action_taken != 0:
            ratio_forward_and_V_hard_left.append(l/total_action_taken)
            epsilons.append(epsilon)

        if total_action_taken2 !=0:
            ratio_forward_action_taken.append(f/total_action_taken2)
            
        episode = episode +1
        print("The episode we're currently in is episode: ", episode)
        publish_all()
        save_to_filename = "%s/Q_Table_episode_%d.csv" % (rospack_path, episode)
        q_table.to_csv(save_to_filename)
        # rospy.sleep(0.1)
        
        plt.clf()
        plt.plot(range(len(ratio_forward_and_V_hard_left)), ratio_forward_and_V_hard_left, label='Ratio Fwd & Left')

        plt.xlabel('Episode')
        plt.ylabel('Ratio Forward & left action taken')
        plt.title('Ratio Forward')
        plt.legend()
        plt.pause(0.01)

    plt.ioff()
    plt.show()    

    return q_table

def laserscan_callback(msg):
    global range_front
    global range_right
    global range_left
    global range_corner
    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left
    
    global front
    global right
    global left
    ranges = msg.ranges
    # You changed the range of the right messages from 275:276 to what it currently is for training on u-turn
    range_front = msg.ranges[0:1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[255:270]  # right FOV (between 300 to 345 degrees)
    range_left=msg.ranges[85:95]
    range_corner = min(msg.ranges[300:302])

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_left,i_left = min( (range_left[i_left],i_left) for i_left in range(len(range_left)) )
# HIT_DISTANCE_THRESHOLD = 0.18          
# VCLOSE_THRESHOLD= 0.30
# CLOSE_THRESHOLD = 0.42
# FAR_THRESHOLD=0.54  
# #The below was 0.65 before running the left turn training.
# LOST_DISTANCE_THRESHOLD =0.7    
    # front states
    if (min_front< 0.3):
        front= "close"
    elif (0.3 <min_front < 0.65):
        front = "normal"
    elif (0.65 < min_front):
        front = "far"
      
    if (min_right <0.20):
        right ="vclose"  
    elif ( 0.20<min_right< 0.35):
        right= "close"
    elif (0.35 <min_right < 0.4):
        right = "normal0"
    elif (0.4 <min_right < 0.55):
        right = "normal1"
    elif (0.55 < min_right ):
        right = "far"


if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service('/gazebo/set_model_state')

    gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    
    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    q_table = Qlearn()
    # rospy.spin()
    print("====== Q TABLE AFTER LEARNING ======")
    # print(q_table)
    print(" ")
