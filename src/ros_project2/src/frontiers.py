#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
import rospy
from std_msgs.msg import String 
import std_msgs
import actionlib
import tf
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Point 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import time
#publishing the marker (a black marker that is created to identify the centroid that the robot is navigating to), published to the goal_marker topic. 
goal_marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=1)
#initializing centroids as points, so they can be accessed with cent1.x and cent1.y (x and y coordinates)
cent1=Point()
cent2=Point()
cent3=Point()
cent4=Point()

prev_cent1=Point()
prev_cent2=Point()
prev_cent3=Point()
prev_cent4=Point()
#This is the parking position where the robot will go when it is done exploring the whole map.
PARKING_POSITION = Point()
PARKING_POSITION.x = -0.7
PARKING_POSITION.y = 0.0
#Below is a function that receives x and y positions of the centroid that the robot should go to. The function adds a black cube to that specific centroid
#the robot is navigating to. The marker (published on the goal_marker topic) will be deleted once that centroid has been explored.
def goal_waypoint(x, y, duration = 8.0, park=False):
    goal_marker = Marker()
    goal_marker.header.frame_id = "map"
    goal_marker.header.stamp = rospy.Time.now()
    if not park:
        goal_marker.type=goal_marker.CUBE
        goal_marker.action=goal_marker.ADD
        goal_marker.scale.x= 0.5
        goal_marker.scale.y=0.5
        goal_marker.scale.z= 0.5
        goal_marker.color.a=1.0
        goal_marker.color.r= 0.0  
        goal_marker.color.g= 0.0
        goal_marker.color.b= 0.0
        goal_marker.pose.orientation.w =1.0
        goal_marker.pose.position.x=x
        goal_marker.pose.position.y=y
        goal_marker.pose.position.z=0
    if park:
        goal_marker.action=goal_marker.DELETE
    goal_marker.id = 1
    goal_marker_pub.publish(goal_marker)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id= "map"
    goal.target_pose.header.stamp= rospy.Time.now()
    
    goal.target_pose.pose.position.x= x
    goal.target_pose.pose.position.y= y
    goal.target_pose.pose.position.z= 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)

    if (duration==-1):
        wait = client.wait_for_result()
    else:
        wait = client.wait_for_result(rospy.Duration(duration)) #Timeout
    rospy.loginfo("Finished")

def callback(data):
    global cent1
    global cent2
    global cent3
    global cent4

    global prev_cent1
    global prev_cent2
    global prev_cent3
    global prev_cent4

    global is_started

    if not len(data.markers):
        return
    #saving the locations of the markers in the variables cent1, cent2, cent3, and cent4. Those markers are identified by their ids
    if not list(filter(lambda x: x.id == 100000, data.markers)):
        cent1 = Point()
        #print("id1")
    if not list(filter(lambda x: x.id == 100001, data.markers)):
        cent2 = Point()
        #print("id2")
    if not list(filter(lambda x: x.id == 100002, data.markers)):
        cent3 = Point()
        #print("id3")
    if not list(filter(lambda x: x.id == 100003, data.markers)):
        cent4 = Point()
        #print("id4")

    for i in range (len(data.markers)):
        if data.markers[i].id== 100000:
            cent1=data.markers[i].pose.position
            prev_cent1=cent1
        if data.markers[i].id==100001:
            cent2=data.markers[i].pose.position
            prev_cent2=cent2
        if data.markers[i].id== 100002:
            cent3=data.markers[i].pose.position
            prev_cent3=cent3
        if data.markers[i].id==100003:
            cent4=data.markers[i].pose.position
            prev_cent4=cent4


def timer_callback(timer):
    global cent1
    global cent2
    global cent3
    global cent4
    global prev_cent1
    global prev_cent2
    global prev_cent3
    global prev_cent4
    global count_timeout
    global is_started

    trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
    #x and y coordinates of the robot. This changes as the robot navigates in the environment
    xrob=trans.transform.translation.x
    yrob=trans.transform.translation.y
    print("xrob ", xrob)
    print("yrob ", yrob)

    distance = []
    d1 = 0
    d2 = 0
    d3 = 0
    d4 = 0
    #appending the locations of the centroids to the list "distance" (lines 148-159)
    if (cent1.x and cent1.y):
        d1 = math.sqrt(((cent1.x-xrob)**2)+((cent1.y-yrob)**2))
        distance.append(d1)
    if (cent2.x and cent2.y):
        d2 = math.sqrt(((cent2.x-xrob)**2)+((cent2.y-yrob)**2))
        distance.append(d2)
    if (cent3.x and cent3.y):
        d3 = math.sqrt(((cent3.x-xrob)**2)+((cent3.y-yrob)**2))
        distance.append(d3)
    if (cent4.x and cent4.y):
        d4 = math.sqrt(((cent4.x-xrob)**2)+((cent4.y-yrob)**2))
        distance.append(d4)

    print("distance", distance)
    if distance:
        is_started = True
        count_timeout = 0                       #reset count timeout
        next_goal = min(distance)               #saving the minimum distance in the variable next_goal 
        print("Go To Nearest")
        if (next_goal == d1):
            print ("Go To d1")
            goal_waypoint(cent1.x, cent1.y)
        if (next_goal == d2):
            print ("Go To d2")
            goal_waypoint(cent2.x, cent2.y)
        if (next_goal == d3):
            print ("Go To d3")
            goal_waypoint(cent3.x, cent3.y)
        if (next_goal == d4):
            print ("Go To d4")
            goal_waypoint(cent4.x, cent4.y)
        print("nearest: %.3f" %(next_goal))
    else:
        count_timeout+=1                        #count if no more update on the distance, and do timeout 
    #if the timeout has been passed, the robot parks at the designated location
    if is_started and count_timeout > 8.0: #timeout
        print ("Park")
        goal_waypoint(PARKING_POSITION.x, PARKING_POSITION.y, -1, True)
        rospy.signal_shutdown("Done.") 

if __name__ == '__main__':

    global count_timeout
    global is_started

    count_timeout = 0
    is_started = 0

    rospy.init_node('markerslocationlistener', anonymous=True)
    #Subscribing to the frontier_markers topic done in the publisher.py file
    rospy.Subscriber("/frontier_markers", MarkerArray,callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    # a second callback function defined created to fill in the list "distance" that has the distances between the robot and each centroid 
    timer = rospy.Timer(rospy.Duration(1/2), timer_callback)
    print("timer called")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    rospy.spin()