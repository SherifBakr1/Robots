#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker


def main():
    rospy.init_node("basic_shapes")

    # 1hz
    r = rospy.Rate(1)

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    shape = Marker.CUBE

    while not rospy.is_shutdown():
        marker = Marker()

        # Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
    
        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes"
        marker.id = 0

        # Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape

        # Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = Marker.ADD

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        marker_pub.publish(marker)

        # Cycle between different shapes
        if (shape == Marker.CUBE):
            shape = Marker.SPHERE
        elif (shape == Marker.SPHERE):
            shape = Marker.ARROW
        elif (shape == Marker.ARROW):
            shape = Marker.CYLINDER
        elif (shape == Marker.CYLINDER):
            shape = Marker.CUBE

        r.sleep()


main()