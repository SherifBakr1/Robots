This is a manual to help the professor/TA grade Sherif Bakr's P2T3.

Start by running the following commands: 

$ roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
$ roslaunch ros_project2 turtlebot3_navigation.launch

This will launch gazebo, Rviz, and you will see the translation and rotation data 
of the robot being logged on the terminal continuously.  


- An additional launch file was created (movebase) to set the DWA parameters to improve the robot's maneuverability in the environment.

- In the terminal, type rosrun ros_project2 publisher.py 
- The above command will activate the frontiers map and the markers 

- The frontiers can be seen when on Rviz the map is switched to frontiers map


- the markers are also shown when the markerarray element is enabled on Rviz

- The  squares represent the centroid of each frontier segment (that are 
represented by different colors)

- you can also notice that, when the robot is moved around using the 2D NAV Goal 
button in Rviz, the frontiers' location are updated on the map. The switching between 
the original map and the frontiers map show that both line up and that the frontiers
show where they are supposed to show. 

-You can also see at the end that when the robot moves, the frontiers and the markers 
update based on the new location of the robot, as required. When the robot arrives
at the designated location, make sure you wait for a few seconds until the 
markers' location and frontier map updates.

- Run the command rosrun_ros_project2 frontiers.py and then go to RVIZ and add a marker, and select the topic (there should only be 1) which enables 
the marker which the robot is navigating to to turn black. 

-The robot will navigate to the closest frontier centroid and then the next closest, etc.

-The robot will then park at its designated location when it is done exploring all the frontiers' centroids.
