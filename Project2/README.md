ENPM661 Project2: To reach the goal node from a start point using BFS or DIJKSTRA on a map of width 400 and height 300.

There are 2 files P2_BFS.py where the start to goal node is reached through the BFS algorithm and P2_DIJKSTRA.py which runs the same using DIJKSTRA algorithm. The time taken to reach the goal state using BFS is 5-6 mins and using Dijkstra the goal state is reached in 10-18 secs which includes backtracking using the parent node information.

The logic for the BFS is as follows. 
1. The start node and goal node is given by the x,y position which takes input from the user
2. A function is called to create a list of the obstacle coordinates and initialize an image which would later be used to setup the animation.
3. There are checks to verify start node is the same as the goal node, if the start ot the goal node is in the obstacle space or outside the map.
4. There are 2 lists one is the visited list (using a set instead of list as the computation is faster) and open list (parent_node in my case).
5. The child nodes are appended to both the open and the visited list and there is a dictionary which stores the parent information for the backtracking
6. The path taken from start to goal is contained in the text file bfspath1.txt and bfspath2.txt and the backtracking from the goal node is contained in the file bfstrack1.txt and bfstrack2.txt. 
7. The pygame code would run the animation once the goal node is reached and the backtrack is completed.
8. The video for the testcase [1,1] as the start and [399,299] can be found in the video P2BFS.avi.


The logic for the Dijkstra is as follows. 
1. The start node and goal node is given by the x,y position which takes input from the user.
2. A function is called to create a list of the obstacle coordinates and initialize an image which would later be used to setup the animation.
3. There are checks to verify start node is the same as the goal node, if the start ot the goal node is in the obstacle space or outside the map.
4. A priority queue is initialized with the cost and the start node. The cost to reach the rest of the points are set to infinity
5. Once the child nodes are obtained they are checked if it in the visited set. If not the new cost is assigned based on the direction the point robot moves in.
6. If the child nodes are in the visited state the new cost is calculated and assigned if the new cost is less than the original cost.
7. The parent node info is stored in a dictionary path_track.
7. The path taken from start to goal is contained in the text file dijpath1.txt and dijpath2.txt and the backtracking from the goal node is contained in the file dijtrack1.txt and dijtrack2.txt. 
8. The pygame code would run the animation once the goal node is reached and the backtrack is completed.
9. The video for the testcase [1,1] as the start and [399,299] can be found in the video P2DIJKSTRA.avi.

Steps to run the program
1. In the terminal clone the repository https://github.com/jayesh68/ENPM661.git or extract the zip folder of the file.
2. Navigate to the folder Project2.
3. Run the command python3 P2_BFS.py and P2_DIJKSTRA.py
4. There are a total of 2 test cases. Each test case needs to be uncommented to run.

The two test cases for BFS are:
1. Start node [1,1] and goal node [399,299].
2. Start node [30,30] and goal node [230,250].

The two test cases for Dijkstra are:
1. Start node [1,1] and goal node [399,299].
2. Start node [50,50] and goal node [200,200].


Have added updated script for BFS P2_BFS_UPDATED.py where redundant code has been removed and the if user inputs invalid values the program restarts and the user needs to reenter those values again.
