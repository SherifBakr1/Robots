#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import math
import numpy as np
import random
global s
global q 
#global states
#global actions 
global rightRange
global frontRange

#states = 5 
#actions = 5 
#q_table =np.zeros((states, actions))

q= []
q0=[0.02,0,0.05,0,0]
q1=[0,0,0.05,0,0]
q2=[0,0.02,0.05,0,0]
q3=[0,0,0,0,0]
q4=[0,0,0,0,0]
 
q.append(q0)
q.append(q1)
q.append(q2)
q.append(q3)
q.append(q4)


# q[0] has the action list that corresponds to state 0
# q[1] has the action list that corresponds to state 1
# q[2] has the action list that corresponds to state 2
# q[3] has the action list that corresponds to state 3
# q[4] has the action list that corresponds to state 4


def callback(msg):

    rightRange= msg.ranges[-135:-45]
    frontRange= msg.ranges[-44:44]

        #I am assuming here that the wall is to the right of the robot (we don't need to worry about the case when the wall is to the left)
    #if msg.ranges[-89] < 0.6 and msg.ranges[0]>0.5:
    if len(rightRange)!=0 and len(frontRange)!=0:
        if (min(rightRange)) < 0.6 and (min(frontRange))>0.5:
            # turn a little bit to the left and keep going forward
            s=0
            qtable(s)
        if min(rightRange) > 0.6 and min(rightRange) < 0.65 and min(frontRange)>0.5:
            # no need to turn left or right, just keep going straight 
            s=1
            qtable(s)
        if min(rightRange) >= 0.65 and min(frontRange)>0.5:
            # turn a little bit to the right and keep going forward 
            s=2 
            qtable(s)
        if min(frontRange)<=0.4:
            #stop
            s=3
            qtable(s)
def qtable(s):
    #Assigning the move commands (actions) for each of the states
        if s == 0:
            move.linear.x= q[0][2]
            move.angular.z= q[0][0]
            pub.publish(move)
            rospy.Rate(0.1)
        if s==1:
            move.linear.x=q[1][2]
            move.angular.z= q[1][0]
            pub.publish(move) 
            rospy.Rate(0.1) 
        if s==2:
            move.linear.x=q[2][2]
            move.angular.z= (q[2][1])*-1
            pub.publish(move) 
            rospy.Rate(0.1)         
        if s==3:
            move.linear.x=q[3][2]
            move.angular.z= q[3][0]
            pub.publish(move)  
            rospy.Rate(0.1)
'''
def learning():

    for episode in range (1000):
        epsilon = 0.9
        learningRate= 0.2
        discountFactor= 0.8
        reward=0
        np.zeros(q) #resetting the q table to zeroes
        done=False  #will be updated when the episode is done

        for i in range (100): #it should be changed to while the robot has not crashed or has not gone far from its trajectory
            exploration_threshold= random.randrange(0,1)
            if exploration_threshold > epsilon:
                action= np.argmax(q[[i],:]) #exploit the environment (highest Q-value)
            else:
                action = q[0].sample()      #explore the environment (pick an action randomly)

        q(i,j)= q (i,j) + learningRate*(reward+ discountFactor*np.max(q[i,:]))

        epsilon -= 0.2

    rewards_per_thousand_episodes = np.split

'''


#initializing the velocity commands node, subscribing to the laser scan topic, and publishing the cmd vel data on the cmd_vel topic
rospy.init_node('velocity_commands', anonymous=True)
sub= rospy.Subscriber('/scan',LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()
rospy.spin()


