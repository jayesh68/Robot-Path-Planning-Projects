This is the folder for the Gazebo simulation of part 1.
1. In part 1 we used a 1000x1000 (100cm x 100 cm) map.
2. The start node and intitial orientation is already entered. 
3. The user needs to input the two RPMs and the goal nodes.
4. The start node of the turtlebot is set in the launch file in terms of the gazebo coordinates. For example if a start node of -4,-4.5 is given in the launch file. The start node in our program would be 100,50.
5. A total of 6 test cases have been tested. For the 10x10 map. The maximum value in gazebo is 5 and the minimum value is -5. 
6. 4 test cases for a start node of 100,50 (gazebo:-4,-4.5) and initial orientation of 30 degrees. 
   (i) For the first test case the goal coordinates to be reached are 500,400 (gazebo: 0,-1). The RPMs used were 30,35.
   (ii) For the second case the goal coordinates are 800,100. The RPMs used were (800,100).
   (iii) For the third test case the goal coordinates were 200,350 (gazebo: -3,-3.5). The RPMs used were 20,30. The magnitude of the velocity had to be scaled by 1.5 in this case      to reach the goal node.
   (iv) For the fourth case the goal coordinates were 300,700 (gazebo: -2,2). The RPMs used were 20,30. The magnitude of the velocity had to be scaled by 1.5 in this case      to    reach the goal node.
7. The remaining 2 test case were:
   (i) FOr a start node of 800,500 (gazebo: 3,0) and the goal node to be 700,700 (gazebo 2,2). The RPMs which worked were 5,10.
   (ii) For a start node of 600,800 (gazebo: 1,3) and the goal node to be 900,900 (gazebo: 4,4). The RPMs which worked were 15,20.

Steps to launch the file in gazebo.
1. Clone the repository to your local machine. 
2. Copy the ROS package p3_Astar to your catkin workspace.
3. The python script would present in the src folder in the ROS package. The name of the file is p3_astar_ros1.py.
4. The file can be launched using the command roslaunch p3_Astar astar.launch

The video simulations of the 6 test cases have been attached in the simulation videos folder
