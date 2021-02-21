ENPM661 Project1: Solving a 16 piece puzzle

The initial state is stored as a 1d list. For example as [1 2 4 5 7 8 9 10 0 11 12 13 15 14 3 6]
The goal state to be reached is [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15]

The program uses the brute force method to reach the goal state. The programs follows the following logic
1. Appends the inital state to an open list and visited list.
2. Has a for loop to traverse through the open list.
3. Finds the location of the blank tile and checks what the possoble movements are for that tile.
4. Checks if the new child has already been visited.
5. If not, append to the visited list and the open list and update a dictionary with the values of the newly obtained child nodes to backtrack the path followed.
6. In the future iterations the new nodes in the open list would be validated based on which the subsequent child nodes are obtained.
7. verify after each iteration if the goal state is reached.
8. If reached write all the visited states by the blank tile into a file called nodePath.txt and all the paths followed from goal to start in a file called nodetrack.txt.


Steps to run the program
1. In the terminal clone the repository https://github.com/jayesh68/ENPM661.git or extract the zip folder of the file.
2. Navigate to the folder Project1.
3. Run the command python3 16 puzzle.py
4. There are a total of 5 test cases which are stored in a variable called s in the program. Each test cases needs to be uncommented to run.

There are a total of 5 test cases.

1. [[1, 2, 3, 4],[ 5, 6,0, 8], [9, 10, 7, 12] , [13, 14, 11, 15]]: All the visited states are stored in the file nodePath1.txt and the paths followed to reach the goal state is stored in nodetrack1.txt.
2. [1, 0, 3, 4],[ 5, 2, 7, 8], [9, 6, 10, 11] , [13, 14, 15, 12]]: All the visited states are stored in the file nodePath2.txt and the paths followed to reach the goal state is stored in nodetrack2.txt.
3. [[0, 2, 3, 4],[ 1,5, 7, 8], [9, 6, 11, 12] , [13, 10, 14, 15]]: All the visited states are stored in the file nodePath3.txt and the paths followed to reach the goal state is stored in nodetrack3.txt.
4. [[5, 1, 2, 3],[0,6, 7, 4], [9, 10, 11, 8] , [13, 14, 15, 12]]: All the visited states are stored in the file nodePath4.txt and the paths followed to reach the goal state is stored in nodetrack4.txt.
5. [[1, 6, 2, 3], [9,5, 7, 4], [0, 10, 11, 8] , [13, 14, 15, 12]]: All the visited states are stored in the file nodePath5.txt and the paths followed to reach the goal state is stored in nodetrack5.txt.


The time taken to run the program is displayed as well.
