#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import *
from std_msgs.msg import *
#import lidar_to_grid_map as lg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

grid = OccupancyGrid()  #initializing variable grid
my_data =None           # initializing an empty data variable

THRESHOLD_CHANGE = 1 #it will set the map_update = true if the different of -1 on the current map and the previous updated value is more than THRESHOLD_CHANGE
prev_map_count_data = 0  #once this number changes, the map will update.
map_updated = False      #boolean variable to check whether or not the map has been updated
#the below empty lists will have the coordinates of each frontier cells in the form x1,y1,x2,y2,...,etc.
first=[]
second=[]
third=[]
fourth=[]
first2=[]
second2=[]
third2=[]
fourth2=[]
#initializing variables for the x and y coordinates of each frontiers' centroid
firstcentroidx =0
firstcentroidy=0
secondcentroidx=0
secondcentroidy=0
thirdcentroidx=0
thirdcentroidy=0
fourthcentroidx=0
fourthcentroidy=0

firstcentroidx2 = 0
firstcentroidy2 = 0
secondcentroidx2 = 0
secondcentroidy2 = 0
thirdcentroidx2 = 0
thirdcentroidy2 = 0
fourthcentroidx2 = 0
fourthcentroidy2 = 0
#publishing the MarkerArray on the topic frontier_markers.
marker_array_pub = rospy.Publisher("/frontier_markers", MarkerArray, queue_size=1)

# state: 0 = not detected, 1 = detected, 2 = finished
state_id1 = 0
state_id2 = 0
state_id3 = 0
state_id4 = 0
state_id12 = 0
state_id22 = 0
state_id32 = 0
state_id42 = 0

def callback(data):
    global prev_map_count_data
    global map_updated
    map_count_data = data.data.count(-1)
    # the following condition updates the map, it checks the new map data count (the number of -1s in the map) and update or doesn't update accordingly
    if (not (abs(prev_map_count_data - map_count_data) >= THRESHOLD_CHANGE)):
        map_updated = False
        return
    else:
        map_updated = True
        print("Updated")
    prev_map_count_data = map_count_data
    #The following 4 variables are used to store the map data in differennt forms, as will be shown later, 
    global my_data     
    global my_data2
    global my_data3
    global my_data4
    global grid
    #w is the width of the grid, h is the height of the grid, r is the resolution of the frid, ox and oy are the origin of the grid
    global w
    global h
    global r
    global ox
    global oy
    
    global first        #Similar to first, I added second, third and fourth which will store the locations of the frontiers.                    
    global second
    global third
    global fourth
    global firstx
    global firsty
    global secondx
    global secondy
    global thirdx
    global thirdy
    global fourthx
    global fourthy

    global first2                           
    global second2
    global third2
    global fourth2
    global firstx2
    global firsty2
    global secondx2
    global secondy2
    global thirdx2
    global thirdy2
    global fourthx2
    global fourthy2

    global firstcentroidx
    global firstcentroidy
    global secondcentroidx
    global secondcentroidy
    global thirdcentroidx
    global thirdcentroidy
    global fourthcentroidx
    global fourthcentroidy

    global firstcentroidx2
    global firstcentroidy2
    global secondcentroidx2
    global secondcentroidy2
    global thirdcentroidx2
    global thirdcentroidy2
    global fourthcentroidx2
    global fourthcentroidy2

    global sumfirstx
    global sumfirsty
    global sumsecondx
    global sumsecondy
    global sumthirdx
    global sumthirdy
    global sumfourthx
    global sumfourthy 

    global sumfirstx2
    global sumfirsty2
    global sumsecondx2
    global sumsecondy2
    global sumthirdx2
    global sumthirdy2
    global sumfourthx2
    global sumfourthy2

    global centfirst
    global centsecond
    global centthird
    global centfourth

    global centfirst2
    global centsecond2
    global centthird2
    global centfourth2
    #initializing x and y coordinates for each centroid
    firstcentroidx =0
    firstcentroidy=0
    secondcentroidx=0
    secondcentroidy=0
    thirdcentroidx=0
    thirdcentroidy=0
    fourthcentroidx=0
    fourthcentroidy=0

    firstcentroidx2 =0
    firstcentroidy2=0
    secondcentroidx2=0
    secondcentroidy2=0
    thirdcentroidx2=0
    thirdcentroidy2=0
    fourthcentroidx2=0
    fourthcentroidy2=0

    first=[]
    second=[]
    third=[]
    fourth=[]
    firstx=[]
    secondx=[]
    thirdx=[]
    fourthx=[]
    firsty=[]
    secondy=[]
    thirdy=[]
    fourthy=[]
    
    first2=[]
    second2=[]
    third2=[]
    fourth2=[]
    firstx2=[]
    secondx2=[]
    thirdx2=[]
    fourthx2=[]
    firsty2=[]
    secondy2=[]
    thirdy2=[]
    fourthy2=[]
    
    w= data.info.width
    h=data.info.height
    r=data.info.resolution
    ox=data.info.origin.position.x
    oy=data.info.origin.position.y
    my_data = list(data.data)
    my_map_metadata= MapMetaData()
    my_map_metadata= data.info
    my_data2=np.array(my_data)
    my_data3= my_data2.reshape((h, w))  #reshaping the map data into w 2D array, it will later be changes into a 1D list and stored in my_data4
    #The following nested loop detects the frontier cells by comparing each cell that has a 0 with the surrounding cells
    for i in range(1, h-2):
        for j in range (1, w-2):
            if (my_data3[i][j]==0 and ((my_data3[i+1][j]==-1 and my_data3[i+2][j]==-1) or (my_data3[i+1][j+1]==-1 and my_data3[i+2][j+2]==-1) or (my_data3[i+1][j-1]==-1 and my_data3[i+2][j-2]==-1) or (my_data3[i-1][j]==-1 and my_data3[i-2][j]==-1) or (my_data3[i-1][j-1]==-1 and my_data3[i-2][j-2]==-1)  or (my_data3[i][j+1]==-1 and my_data3[i][j+2]==-1) or (my_data3[i][j-1]==-1 and my_data3[i][j-2]==-1) or (my_data3[i-1][j+1]==-1 and my_data3[i-2][j+2]==-1))):
                my_data3[i][j]=100
            else:
                my_data3[i][j]=0

    #converting the 2D list into a 1D list and saving it in the variable my_data4
    my_data4=my_data3.flatten()
    grid.data=my_data4
    grid.info=my_map_metadata
    #the below loop segments the frontiers into 4 segments, the locations of each frontier cell will be stored based on its location on the map.
    for i in range(0, int(h/2)+1):
        for j in range (0, int(w/2)+1):
            if (my_data3[i][j]==100 and j< int((w/4)+1)):
                firstx.append(i)
                firsty.append(j)
            if my_data3[i][j]==100 and j>= int((w/4)+1):
                firstx2.append(i)
                firsty2.append(j)

    for i in range(0, int(h/2)+1):
        for j in range (int(w/2)+1, w):
            if (my_data3[i][j]==100 and j< int(3*w/4)):
                secondx.append(i)
                secondy.append(j)
            if my_data3[i][j]==100 and j>= int(3*w/4):
                secondx2.append(i)
                secondy2.append(j)           

    for i in range(int(h/2), h):
        for j in range (0, int(w/2)):
            if (my_data3[i][j]==100 and i < int(3*h/4)) :
                thirdx.append(i)
                thirdy.append(j)
            if (my_data3[i][j]==100 and i >= int(3*h/4) ):
                thirdx2.append(i)
                thirdy2.append(j)

    for i in range(int(h/2), h):
        for j in range (int(w/2), w):
            if (my_data3[i][j]==100 and j < int(3*w/4)) :
                fourthx.append(i)
                fourthy.append(j)
            if (my_data3[i][j]==100 and j >= int(3*w/4) ):
                fourthx2.append(i)
                fourthy2.append(j)

    print(firstx)
    print(firsty)
    print(secondx)
    print(secondy)
    print(thirdx)
    print(thirdy)
    print(fourthx)
    print(fourthy)

    print(firstx2)
    print(firsty2)
    print(secondx2)
    print(secondy2)
    print(thirdx2)
    print(thirdy2)
    print(fourthx2)
    print(fourthy2)
    
    #the below (from line 168-231) calculates the centroid of each frontier segment
    sumfirstx = 0
    for element in range (0, len(firstx)):
        sumfirstx += firstx[element]
    if len(firstx)!=0:
        firstcentroidx= sumfirstx/(len(firstx))
    else:
        firstcentroidx=0

    sumfirstx2 = 0
    for element in range (0, len(firstx2)):
        sumfirstx2 += firstx2[element]
    if len(firstx2)!=0:
        firstcentroidx2= sumfirstx2/(len(firstx2))
    else:
        firstcentroidx2=0

    sumfirsty = 0 
    for element in range (0, len(firsty)):
        sumfirsty += firsty[element]    
    if len(firsty)!=0:
        firstcentroidy= sumfirsty/(len(firsty))
    else:
        firstcentroidy=0

    sumfirsty2 = 0 
    for element in range (0, len(firsty2)):
        sumfirsty2 += firsty2[element]    
    if len(firsty2)!=0:
        firstcentroidy2= sumfirsty2/(len(firsty2))
    else:
        firstcentroidy2=0

    sumsecondx = 0 
    for element in range (0, len(secondx)):
        sumsecondx += secondx[element]       
    if len(secondx)!=0:
        secondcentroidx= sumsecondx/(len(secondx))
    else:
        secondcentroidx=0

    sumsecondx2 = 0 
    for element in range (0, len(secondx2)):
        sumsecondx2 += secondx2[element]       
    if len(secondx2)!=0:
        secondcentroidx2= sumsecondx2/(len(secondx2))
    else:
        secondcentroidx2=0
   
    sumsecondy = 0 
    for element in range (0,len(secondy)):
        sumsecondy+= secondy[element]
    if len(secondy)!=0:
        secondcentroidy= sumsecondy/(len(secondy))
    else:
        secondcentroidy=0

    sumsecondy2 = 0 
    for element in range (0,len(secondy2)):
        sumsecondy2+= secondy2[element]
    if len(secondy2)!=0:
        secondcentroidy2= sumsecondy2/(len(secondy2))
    else:
        secondcentroidy2=0

    sumthirdx = 0
    for element in range (0,len(thirdx)):
        sumthirdx +=  thirdx[element]
    if len(thirdx)!=0:
        thirdcentroidx= sumthirdx/(len(thirdx))
    else:
        thirdcentroidx=0

    sumthirdx2 = 0
    for element in range (0,len(thirdx2)):
        sumthirdx2 +=  thirdx2[element]
    if len(thirdx2)!=0:
        thirdcentroidx2= sumthirdx2/(len(thirdx2))
    else:
        thirdcentroidx2=0

    sumthirdy=0
    for element in range (0,len(thirdy)):
        sumthirdy +=  thirdy[element]   
    if len(thirdy)!=0:
        thirdcentroidy= sumthirdy/(len(thirdy))
    else:
        thirdcentroidy=0

    sumthirdy2=0
    for element in range (0,len(thirdy2)):
        sumthirdy2 +=  thirdy2[element]   
    if len(thirdy2)!=0:
        thirdcentroidy2= sumthirdy2/(len(thirdy2))
    else:
        thirdcentroidy2=0

    sumfourthx = 0
    for element in range (0,len(fourthx)):
        sumfourthx +=  fourthx[element]
    if len(fourthx)!=0:
        fourthcentroidx = sumfourthx/(len(fourthx))
    else:
        fourthcentroidx=0

    sumfourthx2 = 0
    for element in range (0,len(fourthx2)):
        sumfourthx2 +=  fourthx2[element]
    if len(fourthx2)!=0:
        fourthcentroidx2 = sumfourthx2/(len(fourthx2))
    else:
        fourthcentroidx2=0

    sumfourthy=0
    for element in range (0,len(fourthy)):
        sumfourthy +=  fourthy[element]
    if len(fourthy)!=0:
        fourthcentroidy = sumfourthy/(len(fourthy)) 
    else:
        fourthcentroidy=0

    sumfourthy2=0
    for element in range (0,len(fourthy2)):
        sumfourthy2 +=  fourthy2[element]
    if len(fourthy2)!=0:
        fourthcentroidy2 = sumfourthy2/(len(fourthy2)) 
    else:
        fourthcentroidy2=0

    #the following loop adds the coordinates of the frontiers in the form [x1,y1,x2,y2,x3,y3,...]
    for i in range (0, len(firstx)):
        first.append(firstx[i])
        first.append(firsty[i])

    for i in range (0, len(firstx2)):
        first2.append(firstx2[i])
        first2.append(firsty2[i])

    #Similarly, I append to the second, third and fourth lists.
    for i in range (0, len(secondx)):
        second.append(secondx[i])
        second.append(secondy[i])

    for i in range (0, len(secondx2)):
        second2.append(secondx2[i])
        second2.append(secondy2[i])

    for i in range (0, len(thirdx)):
        third.append(thirdx[i])
        third.append(thirdy[i])

    for i in range (0, len(thirdx2)):
        third2.append(thirdx2[i])
        third2.append(thirdy2[i])

    for i in range (0, len(fourthx)):
        fourth.append(fourthx[i])
        fourth.append(fourthy[i])

    for i in range (0, len(fourthx2)):
        fourth2.append(fourthx2[i])
        fourth2.append(fourthy2[i])
#the following function updates the markers based on the updated centroids' positions.
def update_marker():
    global marker_array_pub
    if not map_updated:
        return

    global first
    global second
    global third
    global fourth
    global first2
    global second2
    global third2
    global fourth2
    global firstcentroidx
    global firstcentroidy
    global secondcentroidx
    global secondcentroidy
    global thirdcentroidx
    global thirdcentroidy
    global fourthcentroidx
    global fourthcentroidy
    global firstcentroidx2
    global firstcentroidy2
    global secondcentroidx2
    global secondcentroidy2
    global thirdcentroidx2
    global thirdcentroidy2
    global fourthcentroidx2
    global fourthcentroidy2
    global prev_centroid_1
    global prev_centroid_2
    global prev_centroid_3
    global prev_centroid_4
    global prev_centroid_12
    global prev_centroid_22
    global prev_centroid_32
    global prev_centroid_42

    id = 0
    
    if (len(first) <=1 and len(second) <=1 and len(third) <=1 and len(fourth) <=1 ):
        return
    # if (len(first2) <=1 and len(second2) <=1 and len(third2) <=1 and len(fourth2) <=1 ):
    #     return
    global state_id1
    global state_id2
    global state_id3
    global state_id4
    global state_id12
    global state_id22
    global state_id32
    global state_id42
    marker_array = MarkerArray()

    marker_array_clear = MarkerArray()
    marker_clear = Marker()
    marker_clear.header.frame_id="map"
    marker_clear.header.stamp=rospy.Time.now()
    marker_clear.action = marker_clear.DELETEALL
    marker_array.markers.append(marker_clear)
    marker_array_pub.publish(marker_array_clear)
    #print(first)
    #print(second)
    #print(third)
    #print(fourth)
    #print(first2)
    #print(second2)
    #print(third2)
    #print(fourth2)
    if (len(first) > 1):
        for i in range(0, len(first)-1, 2):   #creates a marker for each frontier cell. The same approach is repeated for each segment (lines 294-524)
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a=1.0
            marker.color.r= 1.0 
            marker.color.g= 0.0
            marker.color.b= 0.0
            marker.pose.orientation.w =1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((first[i+1])*r)+ oy 
            marker.pose.position.y=((first[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW! creating a marker for the centroid.
        if firstcentroidx!=0 and firstcentroidy!=0 and len(first)>50:
            state_id1 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 1.0  
            marker_centroid.color.g= 0.0
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(firstcentroidy*r)+ oy 
            marker_centroid.pose.position.y=(firstcentroidx*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100000
            rospy.loginfo("first centroid")
            marker_array.markers.append(marker_centroid)
            
            prev_centroid_1 = [((firstcentroidy*r)+ oy), ((firstcentroidx*r)+ ox)]

        else:  #deleting the centroid if the centroid has been discovered in the updated map. The same process has been repeated for centroids 2nd, 3rd, and 4th.
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100000
            
            marker_array.markers.append(marker_centroid)

    if (len(first2) > 1):
        for i in range(0, len(first2)-1, 2):   #creates a marker for each frontier cell. The same approach is repeated for each segment (lines 294-524)
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a=1.0
            marker.color.r= 1.0 
            marker.color.g= 0.0
            marker.color.b= 0.0
            marker.pose.orientation.w =1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((first2[i+1])*r)+ oy 
            marker.pose.position.y=((first2[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW! creating a marker for the centroid.
        if firstcentroidx2!=0 and firstcentroidy2!=0 and len(first2)>50:
            state_id12 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 1.0  
            marker_centroid.color.g= 0.0
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(firstcentroidy2*r)+ oy 
            marker_centroid.pose.position.y=(firstcentroidx2*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100010
            rospy.loginfo("first centroid 2")
            marker_array.markers.append(marker_centroid)
            
            prev_centroid_12 = [((firstcentroidy2*r)+ oy), ((firstcentroidx2*r)+ ox)]

        else:  #deleting the centroid if the centroid has been discovered in the updated map. The same process has been repeated for centroids 2nd, 3rd, and 4th.
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100010
            
            marker_array.markers.append(marker_centroid)

    if (len(second) > 1):
        for i in range(0, len(second)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.0
            marker.color.g= 1.0
            marker.color.b= 0.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((second[i+1])*r)+ oy 
            marker.pose.position.y=((second[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if secondcentroidx!=0 and secondcentroidy!=0 and len(second)>40:
            state_id2 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.0
            marker_centroid.color.g= 1.0
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(secondcentroidy*r)+ oy 
            marker_centroid.pose.position.y=(secondcentroidx*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100001
            rospy.loginfo("second centroid")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_2 = [((secondcentroidy*r)+ oy), ((secondcentroidx*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100001
            
            marker_array.markers.append(marker_centroid)

    if (len(second2) > 1):
        for i in range(0, len(second2)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.0
            marker.color.g= 1.0
            marker.color.b= 0.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((second2[i+1])*r)+ oy 
            marker.pose.position.y=((second2[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if secondcentroidx2!=0 and secondcentroidy2!=0 and len(second2)>40:
            state_id22 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.0
            marker_centroid.color.g= 1.0
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(secondcentroidy2*r)+ oy 
            marker_centroid.pose.position.y=(secondcentroidx2*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100011
            rospy.loginfo("second centroid2")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_22 = [((secondcentroidy2*r)+ oy), ((secondcentroidx2*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100011
            
            marker_array.markers.append(marker_centroid)

    if (len(third) > 1):
        for i in range(0, len(third)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.0
            marker.color.g= 0.0
            marker.color.b= 1.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((third[i+1])*r)+ oy 
            marker.pose.position.y=((third[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if thirdcentroidx!=0 and thirdcentroidy!=0 and len(third)>40:
            state_id3 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.0 
            marker_centroid.color.g= 0.0
            marker_centroid.color.b= 1.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(thirdcentroidy*r)+ oy 
            marker_centroid.pose.position.y=(thirdcentroidx*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100002
            rospy.loginfo("third centroid")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_3 = [((thirdcentroidy*r)+ oy), ((thirdcentroidx*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100002
            
            marker_array.markers.append(marker_centroid)

    if (len(third2) > 1):
        for i in range(0, len(third2)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.0
            marker.color.g= 0.0
            marker.color.b= 1.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((third2[i+1])*r)+ oy 
            marker.pose.position.y=((third2[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if thirdcentroidx2!=0 and thirdcentroidy2!=0 and len(third2)>40:
            state_id32 = 1
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.0 
            marker_centroid.color.g= 0.0
            marker_centroid.color.b= 1.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(thirdcentroidy2*r)+ oy 
            marker_centroid.pose.position.y=(thirdcentroidx2*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100012
            rospy.loginfo("third centroid2")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_32 = [((thirdcentroidy2*r)+ oy), ((thirdcentroidx2*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100012
            
            marker_array.markers.append(marker_centroid)

    if (len(fourth) > 1):
        for i in range(0, len(fourth)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.5
            marker.color.g= 0.5
            marker.color.b= 0.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((fourth[i+1])*r)+ oy 
            marker.pose.position.y=((fourth[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if fourthcentroidx!=0 and fourthcentroidy!=0 and len(fourth)>50:
            state_id4 = 1

            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.5
            marker_centroid.color.g= 0.5
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(fourthcentroidy*r)+ oy 
            marker_centroid.pose.position.y=(fourthcentroidx*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100003
            rospy.loginfo("fourth centroid")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_4 = [((fourthcentroidy*r)+ oy), ((fourthcentroidx*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100003
            
            marker_array.markers.append(marker_centroid)
    
    if (len(fourth2) > 1):
        for i in range(0, len(fourth2)-1, 2):
            marker=Marker()
            marker.header.frame_id= "map"
            marker.header.stamp=rospy.Time.now()
            marker.type= marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x= 0.1
            marker.scale.y=0.1
            marker.scale.z= 0.1
            marker.color.a= 1.0
            marker.color.r= 0.5
            marker.color.g= 0.5
            marker.color.b= 0.0
            marker.pose.orientation.w = 1.0
            marker.id = id
            id += 1
            marker.pose.position.x=((fourth2[i+1])*r)+ oy 
            marker.pose.position.y=((fourth2[i])*r)+ ox 
            marker.pose.position.z=0

            marker_array.markers.append(marker)

        #################################CENTROID BELOW!creating a marker for the centroid.
        if fourthcentroidx2!=0 and fourthcentroidy2!=0 and len(fourth2)>50:
            state_id42 = 1

            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.MODIFY
            marker_centroid.scale.x= 0.4
            marker_centroid.scale.y=0.4
            marker_centroid.scale.z= 0.4
            marker_centroid.color.a=1.0
            marker_centroid.color.r= 0.5
            marker_centroid.color.g= 0.5
            marker_centroid.color.b= 0.0
            marker_centroid.pose.orientation.w =1.0
            marker_centroid.pose.position.x=(fourthcentroidy2*r)+ oy 
            marker_centroid.pose.position.y=(fourthcentroidx2*r)+ ox 
            marker_centroid.pose.position.z=0
            marker_centroid.id = 100013
            rospy.loginfo("fourth centroid2")
            
            marker_array.markers.append(marker_centroid)
            prev_centroid_42 = [((fourthcentroidy2*r)+ oy), ((fourthcentroidx2*r)+ ox)]

        else:
            marker_centroid=Marker()
            marker_centroid.header.frame_id= "map"
            marker_centroid.header.stamp=rospy.Time.now()
            marker_centroid.type= marker_centroid.CUBE
            marker_centroid.action=marker_centroid.DELETE
            marker_centroid.id = 100013
            
            marker_array.markers.append(marker_centroid)
    
    #the following publishes the marker array if the marker array is not empty
    if len(marker_array.markers):
       marker_array_pub.publish(marker_array)

def talker():
    #subscribing to the occupancy grid from the map topic
    rospy.Subscriber("/map", OccupancyGrid, callback)
    #publishing the new occupancy grid map to the frontier_map topic
    pub = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=10)
    #initiating a new node.
    rospy.init_node('frontier_marker_publisher', anonymous=True)

    rospy.loginfo('starting node')

    rate = rospy.Rate(10) 
    #publshing the grid, updating the marker using the function, until rospy is shutdown
    while not rospy.is_shutdown():
        rospy.get_time()
        pub.publish(grid)
        update_marker()
        rate.sleep()
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e)
        pass