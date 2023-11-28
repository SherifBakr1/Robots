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
action_rotate_90_deg_pub                    = rospy.Publisher('action_rotate_90_deg', Float32, queue_size=1)
action_rotate_180_pub                       = rospy.Publisher('action_rotate_180_deg', Float32, queue_size=1)
episode_pub                                 = rospy.Publisher('episode', Int64, queue_size=1)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
HIT_DISTANCE_THRESHOLD = 0.24 #hit
LOST_DISTANCE_THRESHOLD = 0.36 #very far

FRONT_CLOSE_THRESHOLD = 0.4
FRONT_FAR_THRESHOLD = 0.75

CLOSE_THRESHOLD = 0.3
# NORMAL_THRESHOLD = 0.6
FAR_THRESHOLD = 0.35
# VERY_FAR_THRESHOLD = 0.4

N_STATES = 5
N_ACTIONS = 5

# Initialize all variables
range_front = []
range_right = []
range_left  = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_right_wide = 0
i_right_wide = 0
min_left = 0
i_left = 0
front = ""
left = ""
right = ""
state = 0

# Fixing the state of the 180 degree turn 
# plotting the graph of CORRECT FORWARD ACTIONS / TOTAL ACTIONS
# plotting the graph of CORRECT RIGHT (180 DEGREE TURN) ACTIONS / TOTAL ACTIONS
# plotting the graph of CORRECT LEFT ACTIONS (STRAIGHT, AND THE 90 DEGREE TURN) / TOTAL ACTIONS



ACTIONS     = ['FORWARD_AND_TURN_LITTLE_LEFT', 'FORWARD', 'FORWARD_AND_TURN_LITTLE_RIGHT', '90_DEG_LEFT_TURN', '180_DEG_LEFT_TURN']
STATES      = ['RIGHT_WALL_CLOSE', 'RIGHT_WALL_MEDIUM', 'RIGHT_WALL_FAR', 'FRONT_WALL_MEDIUM', 'RIGHT_WALL_SUDDENLY_NOT_FOUND'] 
N_STATES    = len(STATES)

MAX_EPISODE = 1000   # DURING THE DEVELOPMENT/IMPLEMENTATION PLEASE FEEL FREE TO CHANGE THIS VALUE
GAMMA       = 0.9   # discount rate
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
forward_with_turn_little_left       = 0
forward                             = 0
forward_with_turn_little_right      = 0
rotate_90_deg                       = 0
rotate_180_deg                      = 0

ratio_forward_action_taken                  = []  
ratio_forward_lil_left_action_taken          = []  
ratio_forward_lil_right_action_taken          = []  
ratio_90deg_action_taken                    = []  
ratio_180deg_action_taken                   = []  


def build_q_table():
    global N_STATES
    global ACTIONS
    table = pd.DataFrame(
        np.zeros((N_STATES, len(ACTIONS)), dtype=float),
        columns=ACTIONS
    )
    return table

def actor(observation, q_table):
    random = np.random.uniform()
    if random > epsilon:
        state_action = q_table.loc[observation, :]
        # action = np.argmax(q_table.loc[observation,:])
        # action = state_action.idxmax()
        action = np.random.choice(state_action[state_action == np.max(state_action)].index)
        print("[Greater Than] State: %d - Random: %.03f > Epsilon: %.03f. Choose max: %s" % (observation, random, epsilon, action))

    else:
        action = np.random.choice(ACTIONS)
        print("[Less Than] State: %d - Random: %.03f < Epsilon: %.03f. Choose random: %s" % (observation, random, epsilon, action))

    return action

def move_robot(speed_linear_x, speed_angular_z):
    msg = Twist()
    msg.linear.x = speed_linear_x
    msg.angular.z = speed_angular_z
    cmd_vel_pub.publish(msg)
  
def move_robot_by_action(action):
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global rotate_90_deg
    global rotate_180_deg
    if action == 'FORWARD_AND_TURN_LITTLE_LEFT':
        move_robot(0.3, 0.2)
        time.sleep(0.1)
        # time.sleep(0.2)
        forward_with_turn_little_left += 1
    elif action == 'FORWARD':
        move_robot(0.3, 0.0)
        # time.sleep(0.2)
        forward += 1
    elif action == 'FORWARD_AND_TURN_LITTLE_RIGHT':
        move_robot(0.3, -0.2)
        time.sleep(0.1)
        # time.sleep(0.2)
        forward_with_turn_little_right += 1
    elif action == '90_DEG_LEFT_TURN':
        move_robot(0.0, 0.9)
        # time.sleep(0.3)
        rotate_90_deg += 1
    elif action == '180_DEG_LEFT_TURN':
        move_robot(0.6, -2.0)
        time.sleep(0.2)

        # time.sleep(0.3)

        rotate_180_deg += 1

def publish_all():
    global episode
    global total_action_taken
    global forward_with_turn_little_left
    global forward
    global forward_with_turn_little_right
    global rotate_90_deg
    global rotate_180_deg

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
        msg.data = rotate_90_deg                        / total_action_taken
        action_rotate_90_deg_pub.publish(msg)
        msg.data = rotate_180_deg                       / total_action_taken
        action_rotate_180_pub.publish(msg)

def init_env(episode, total_episode):
    threshold = math.ceil(total_episode / 3)
    print(threshold)

    if episode <= threshold:
        world_x = 0.3
        world_y = -2.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.0
        world_orientation_w = 1

    elif (threshold < episode < (2 * threshold)):
        world_x = 2.0
        world_y = -1.0
        world_orientation_x = 0.0
        world_orientation_y = 0.0
        world_orientation_z = 0.7
        world_orientation_w = 0.7

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
    end = False                                         #WHY DO YOU HAVE THIS?

    if (right=="close" and front!="close"):
        state = 0                                       # turn lil bit left and move fwd
    elif (right=="normal" and front=="far"):
        state = 1                                       # forward
    elif (right=="far" and front =="far"):
        state = 2                                       # turn lil bit right
    elif (front == "close" and right!="far"):
        state = 3                                       # turn 90 deg
    elif (right=="uturn"):                           
        state = 4           
    
    # print("min_front %.03f min_right %.03f STATE %d" % (min_front, min_right, state))
    # print("STATE ", state)
    if min_front <= HIT_DISTANCE_THRESHOLD or min_right_wide <= HIT_DISTANCE_THRESHOLD or min_right > LOST_DISTANCE_THRESHOLD:
        rospy.loginfo("Crashed the wall/Too far - Resetting Gazebo World")
        reward = -1
        end = True
    else:
        reward = 1
 
    return state, reward, end

### Qlearning algoritm
def Qlearn():
    global left
    global front
    global right
    global total_training_rewards
    global total_action_taken
    global episode
    global state
    global f
    global fl
    global epsilon

    reward = 0
    f=0
    fl=0
    q_table = build_q_table()
    episode = 331
    while episode < MAX_EPISODE:
        if rospy.is_shutdown():
            break
        end = init_env(episode, MAX_EPISODE)
        move_robot(0.0, 0.0)
        time.sleep(0.1)
        
        total_training_rewards = 0
        
        output = get_state(front, right)                       
        state = output[0]
        print ("EPISODE : %d" % episode)
        while not end and not rospy.is_shutdown():
            act = actor(state, q_table)
            # print(act)
            move_robot_by_action(act)

            time.sleep(0.3) #time sleep 0.2 is okay
            move_robot(0.0, 0.0)
            time.sleep(0.1)

            next_state, reward, end = get_state(front, right)
            
            q_predict = q_table.loc[state, act]

            ### Qlearning algoritm
            ###################################################################
            
            # if reward == -1:
            #     q_target = reward
            # else:
            q_target = reward + (GAMMA * q_table.iloc[next_state].max())


            q_table.loc[state, act] += ALPHA * (q_target - q_predict)
            state = next_state

            total_training_rewards += reward
            total_action_taken += 1
            # print("TOTAL ACTION TAKEN: ", total_action_taken)
            if state==1:
                f+=1
            if state==2:
                fl+=1

        print(q_table)
        move_robot(0.0, 0.0)
        epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay*(episode/(MAX_EPISODE/3)))

        #Adding the total reward
        training_rewards.append(total_training_rewards)
        
        if episode <= math.ceil(MAX_EPISODE / 3):
            if total_action_taken != 0:
                ratio_forward_action_taken.append(f/total_action_taken)
                ratio_forward_lil_left_action_taken.append(fl/total_action_taken)
                ratio_forward_lil_right_action_taken.append(forward_with_turn_little_right/total_action_taken)
                ratio_90deg_action_taken.append(rotate_90_deg/total_action_taken)
                ratio_180deg_action_taken.append(rotate_180_deg/total_action_taken)
        
            epsilons.append(epsilon)
        # update_line(hl, episode, training_rewards)
        # plt.pause(0.05)
        episode = episode +1
        publish_all()
        # save Q table along with episode (episode, q_table)
        save_to_filename = "%s/Q_Table_episode_%d.csv" % (rospack_path, episode)
        q_table.to_csv(save_to_filename)
        # rospy.sleep(0.1)
        if episode == math.ceil(MAX_EPISODE / 3) + 1:
            print("Switch environment to 2")
            epsilon = 0.9 #reset epsilon to 0.9
            total_action_taken = 0 # reset to 0
        elif episode ==  2 * (math.ceil(MAX_EPISODE / 3)) + 1:
            print("Switch environment to 3")
            epsilon = 0.9 #reset epsilon to 0.9
            total_action_taken = 0 # reset to 0


    print (q_table)
    #Visualizing results and total reward over all episodes
    x = range(episode)
    plt.figure(figsize=(10,6))
    # plt.plot(x, training_rewards)
    # plt.xlabel('Episode')
    # plt.ylabel('Training total reward')
    # plt.title('Total rewards over all episodes in training') 

    plt.plot(x, ratio_forward_action_taken)
    plt.xlabel('Episode')
    plt.ylabel('Ratio Forward action taken')
    plt.title('Ratio Forward action taken all episodes in training') 
    plt.show()    
    return q_table

def laserscan_callback(msg):
    global reward
    global prior_state_
    global state

    global range_front
    global range_right
    global ranges
    global min_front,i_front, min_right,i_right
    global min_right_wide,i_right_wide
    
    global front
    global right
    ranges = msg.ranges
    
    # get the range FOV (Field of View)
    range_front[:20] = msg.ranges[20:0:-1]  # Front1 FOV (between 5 to -5 degrees)
    range_front[20:] = msg.ranges[-1:-20:-1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[240:300]  # right FOV (between 300 to 345 degrees)
    range_right_wide = msg.ranges[240:340]  # right FOV (between 300 to 345 degrees)

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_right_wide,i_right_wide = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )

    # front states
    if (min_front <= FRONT_CLOSE_THRESHOLD):
        front = "close"
    elif (FRONT_CLOSE_THRESHOLD < min_front < FRONT_FAR_THRESHOLD):
        front = "medium"
    elif (min_front >= FRONT_FAR_THRESHOLD):
        front = "far"

    # right states
    if ( min_right <= CLOSE_THRESHOLD):
        right = "close"
    elif (CLOSE_THRESHOLD < min_right < FAR_THRESHOLD):
        right = "normal"
    elif (min_right >= FAR_THRESHOLD):
        right = "far"

    half_right_front = [i for i in msg.ranges[290:320] if i != "inf"]
    half_right_front = max(half_right_front)
    half_right_rear = [i for i in msg.ranges[225:250] if i != "inf"]
    half_right_rear = min(half_right_rear)
    # if (msg.ranges[300] > 10* msg.ranges[225]):
    if (half_right_front > 10* half_right_rear):
        right = "uturn"

if __name__ == "__main__":
    rospy.init_node('wall_follower_node', anonymous=True)

    rospy.wait_for_service("/gazebo/reset_world")
    rospy.wait_for_service('/gazebo/set_model_state')

    gazebo_reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    gazebo_set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    
    rospy.Subscriber("/scan", LaserScan, laserscan_callback, queue_size=1)

    rate = rospy.Rate(15.0)

    rospy.sleep(3.0)

    q_table = Qlearn()

    # init_env(0,MAX_EPISODE)
    # while not rospy.is_shutdown():
    #     state,reward,end = get_state(front, right)
    #     if (state == 0):
    #         action = "FORWARD_AND_TURN_LITTLE_LEFT"
    #     elif (state == 1):
    #         action = "FORWARD"
    #     elif (state == 2):
    #         action = "FORWARD_AND_TURN_LITTLE_RIGHT"
    #     elif (state == 3):
    #         action = "90_DEG_LEFT_TURN"
    #     elif (state == 4):
    #         action = "180_DEG_LEFT_TURN"
    #     print("STATE: %d - Action: %s" %(state, action))
    #     move_robot_by_action(action)
    #     get_state(front, right)
    # rospy.spin()
    # print("====== Q TABLE AFTER LEARNING ======")
    # print(q_table)
    print(" ")