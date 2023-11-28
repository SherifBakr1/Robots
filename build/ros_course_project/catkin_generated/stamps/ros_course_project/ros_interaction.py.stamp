#!/usr/bin/env python3

from gui import GUI
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



class ROSInteraction:
    def __init__(self):
        self.gui = GUI(
            direction_key_callback=self.direction_key_callback,
            coordinate_callback=self.coordinate_callback,
        )

        self.init_node()
        self.init_publisher()
        self.init_subscriber()
        self.init_action_client()

    def init_node(self):
        ###
        ### TODO: Initialize the ROS node here
        ###
        pass

    def init_publisher(self):
        ###
        ### TODO: Initialize the publisher here
        ###
        pass

    def init_subscriber(self):
        ###
        rospy.Subscriber('odom',nav_msgs/Odometry,pose)        
        ###
        pass

    def init_action_client(self):
        ###
        ### TODO: Initialize the action client here
        ###
        pass

    def direction_key_callback(self, direction):
        ###
        ### TODO: Use the publisher to send the direction to the robot
        ###
        pass

    def coordinate_callback(self, x, y, z):
        ###
        ### TODO: Use the action client to send the coordinate to the robot
        ###
        pass

    def subscription_callback(self, odometry):
        pose = odometry.pose.pose
        self.gui.update_position(pose.x, pose.y, pose.z)
