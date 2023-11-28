import rospy
from sensor_msgs.msg import LaserScan

def laserscan_callback(msg):
    # Extract the lidar data

    # Define the range corresponding to the right 5 degrees
    #right_start_index = len(ranges) - 9  # Assuming the lidar data has 360 degrees

    # Extract the right 5 degrees data
    #right_5_degrees_data = ranges[right_start_index:]

    # Print or process the right 5 degrees lidar data
    #print("Right", msg.ranges[270:271])
    ##print("Front", msg.ranges[0:1])
    print("Back: ", msg.ranges[180:181] )
    #if min(msg.ranges[330:331]) > 2:
    print("Laila 3elwy")


    range_right = msg.ranges[255:270]  # right FOV (between 300 to 345 degrees)
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )

    print("min right: ", min_right)

    range_front =msg.ranges[0:1]
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )

    print("min front: ", min_front)

if __name__ == "__main__":
    rospy.init_node('lidar_processing_node', anonymous=True)

    # Subscribe to the /scan topic
    rospy.Subscriber("/scan", LaserScan, laserscan_callback)

    # Spin to keep the script from exiting
    rospy.spin()