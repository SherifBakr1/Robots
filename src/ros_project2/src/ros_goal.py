#!/usr/bin/env python3

from gui import GUI
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import move_base_msgs
import tf
import roslib
#roslib.load_manifest('learning_tf')
import math
import geometry_msgs.msg
import turtlesim.srv
#! /usr/bin/env python

def move_base_client():
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id= "map"
    goal.target_pose.header.stamp= rospy.Time.now()
    #goal.target_pose.pose.position.x= 1.0
    #goal.target_pose.pose.position.y= 0.0
    goal.target_pose.pose.position.x= 1.2
    goal.target_pose.pose.position.y= 0.1
    goal.target_pose.pose.position.z= 0.0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)  
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = move_base_client()
        print("Traveling!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)































#def movebase_client():
#
#    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#    client.wait_for_server()

#    goal = MoveBaseGoal()
#    goal.target_pose.header.frame_id = "map"
#    goal.target_pose.header.stamp = rospy.Time.now()
#    goal.target_pose.pose.position.x = 1.0
#    goal.target_pose.pose.position.y = 1.0
#    goal.target_pose.pose.position.z = 1.0
#    goal.target_pose.pose.orientation.w = 0.0

#    client.send_goal(goal)
#    wait = client.wait_for_result()
#    if not wait:
#        rospy.logerr("Action server not available!")
#        rospy.signal_shutdown("Action server not available!")
#    else:
#        return client.get_result()

#if __name__ == '__main__':
#    try:
#        rospy.init_node('movebase_client_py')
#        result = movebase_client()
#        if result:
#            rospy.loginfo("Goal execution done!")
#    except rospy.ROSInterruptException:
#        rospy.loginfo("Navigation test finished.")

