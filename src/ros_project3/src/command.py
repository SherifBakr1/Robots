#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

global s
global q 
q= []
q0= [0,0,0]
q1=[0,0,0]
q2=[0,0,0]
q3=[0,0,0]
q.append(q0)
q.append(q1)
q.append(q2)
q.append(q3)
# q[0] has the action list that corresponds to state 0
# q[1] has the action list that corresponds to state 1
# q[2] has the action list that corresponds to state 2
# q[3] has the action list that corresponds to state 3
def callback(msg):
    #print (msg.ranges[270])
    #print ("The front reading is", msg.ranges[0])
    print ("The right reading is",msg.ranges[-90])
    ##print ("the FRONT RIGHT READING IS", msg.ranges[291])
    #print("the left reading is" ,msg.ranges[90])
    #print("the back readins is", msg.ranges[180])



    '''global range_front
    global range_right
    global range_left
    global range_front_right

    global ranges
    global min_front,i_front, min_right,i_right, min_left, i_left, min_front_right, i_front_right
    
    global front
    global right
    global left
    global rightfront
    range_front[:15] = msg.ranges[15:0:-1]  # Front1 FOV (between 5 to -5 degrees)
    range_front[15:] = msg.ranges[-1:-15:-1] # Front2 FOV (between 5 to -5 degrees)
    range_right = msg.ranges[225:290]  # right FOV (between 300 to 345 degrees)
    range_front_right= msg.ranges[291:320]
    range_left=msg.ranges[85:95]

    # find the shortest obstacle of each side 
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_front_right,i_front_right = min( (range_front_right[i_front_right],i_front_right) for i_front_right in range(len(range_front_right)) )


    print (min_front_right)
'''
        #I am assuming here that the wall is to the right of the robot (we don't need to worry about the case when the wall is to the left)
    if msg.ranges[-89] < 0.6 and msg.ranges[0]>0.5:
        # turn a little bit to the left and keep going forward
        s=0
        qtable(s)
    if msg.ranges[-89] > 0.6 and msg.ranges[-89] < 0.65 and msg.ranges[0]>0.5:
        # no need to turn left or right, just keep going straight 
        s=1
        qtable(s)
    if msg.ranges [-89] >= 0.65 and msg.ranges[0]>0.5:
        # turn a little bit to the right and keep going forward 
        s=2 
        qtable(s)
    if msg.ranges[0]<=0.4:
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
#initializing the velocity commands node, subscribing to the laser scan topic, and publishing the cmd vel data on the cmd_vel topic
rospy.init_node('velocity_commands', anonymous=True)
sub= rospy.Subscriber('/scan',LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()
rospy.spin()























'''
 el angular.z el positive betlef el robot shemal
    def callback(msg):
    #print (msg.ranges[0]) #the center of the robot between the wheels is the zero degrees
    #print (msg.ranges[89]) # the robot's left
    #print (msg.ranges[-89]) # the robot's right
    #print (msg.ranges[179])
    #print (msg.ranges[359])
    #move.linear.x=0.01
    #if msg.ranges[0] <1:
    #move.linear.x=0
    #pub.publish(move)
    
    #move.angular.z = 1
    #move.linear.x =1

    #pub.publish(move) 
    #rospy.Rate(1)
    ------------------------------------

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('velocity_commands', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist=Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        sub= rospy.Subscriber('/scan',LaserScan, callback)

        rate.sleep()

def callback(msg):
    current_time= rospy.Time.now()
    msg.header.stamp= current_time
    #print(msg.angle_min)
    #print(msg.angle_max)
    #print(msg.angle_increment)
    #print(msg.range_min)
    #print(msg.range_max)
    print(len(msg.ranges))


#def velcom():


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

'''