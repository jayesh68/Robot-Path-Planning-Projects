import numpy as np
import copy
import time
import ast

#Function to find where the blank tile (0) is located and returns row and column
def BlankTileLocate(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	for i in range(len(curr_node1)):
		for j in range(len(curr_node1[i])):
			if curr_node1[i][j] == 0:
				row = i
				col = j
	return([row, col])

#Function to move the blank tile to the left
def ActMoveLeft(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
    #If left movement is possible return the new node
	if c != 0:
		curr_node1[r][c-1], curr_node1[r][c] = curr_node1[r][c], curr_node1[r][c-1]
		return curr_node1
    #If left movement is not possible return the same node
	else:
		return curr_node1

#Function to move the blank tile to the right
def ActMoveRight(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node)[1]
	#If right movement is possible return the new node
	if c != 3:
		curr_node1[r][c+1], curr_node1[r][c] = curr_node1[r][c], curr_node1[r][c+1]
		return curr_node1
    #If right movement is not possible return the same node 
	else:
		return curr_node1

#Function to move the blank tile to the top
def ActMoveUp(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
    #If up movement is possible return the new node
	if r != 0:
		curr_node1[r-1][c], curr_node1[r][c] = curr_node1[r][c], curr_node1[r-1][c]
		return curr_node1
    #If up movement is not possible return the same node 
	else:
		return curr_node1

#Function to move the blank tile to the bottom
def ActMoveDown(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
	#If up movement is possible return the new node
	if r != 3:
		curr_node1[r+1][c], curr_node1[r][c] = curr_node1[r][c], curr_node1[r+1][c]
		return curr_node1
    #If down movement is not possible return the same node 
	else:
		return curr_node1

#Converting the 1d list to a 4x4 matrix to validate right,left,top and bottom movement
def PrintMatrix(k):  
    list_of_num = np.array([k])
    # print('knum',list_of_num)
    if len(list_of_num) == 16:
        for i in range(0, len(list_of_num), 4):
            k.append(list_of_num[i])  
            k.append(list_of_num[i+1]) 
            k.append(list_of_num[i+2])
            k.append(list_of_num[i+3])
            i += 1 
    k = np.reshape(k, (4, 4))  
    # print('k',k)
    return(k) 

#Converting the obtained child nodes to 1d list to check if it is in the visited nodes or not
def Mat2List(m):
    p = []
    for i in m.tolist():
            p = p + i
    return p

# The different input states of test cases 1-5 stored as a 1d list
s = [1,2,3,4,5,6,0,8,9,10,7,12,13,14,11,15]
#s=[1,0,3,4,5,2,7,8,9,6,10,11,13,14,15,12]
#s=[0,2,3,4,1,5,7,8,9,6,11,12,13,10,14,15]
#s=[5,1,2,3,0,6,7,4,9,10,11,8,13,14,15,12]
#s=[1,6,2,3,9,5,7,4,0,10,11,8,13,14,15,12]
g = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0]
start_time = time.time()
visited_nodes = []          #List consisting of all the nodes traversed by the blank tile (0)
parent_node = []            #Open list to which the possible new child nodes would be appended to
parent_node.append(s)       #Appending inital state
child_node = []             #stores the child states after blank tile moves to different positions
path_track={}               #Dictionary storing the values of the different states to backtrack the path followed
print(s)                    
print(g)
flag = 'n'                  #Flag to verify if the goal state is reached
visited_nodes.append(s)     #Appending the inital state to the list of visited nodes
if s == g:
	print('goal already reached')
    
else:
    #traversing through each node in the open list
	for a in parent_node:
        #Calling the functions to obtain the blank tile(0) and validate if the tile can move in the different directions
		l_child_node = ActMoveLeft(PrintMatrix(a))
		u_child_node = ActMoveUp(PrintMatrix(a))
		r_child_node = ActMoveRight(PrintMatrix(a))
		d_child_node = ActMoveDown(PrintMatrix(a))

        #Calling the function to convert the matric to a 1d list
		l_child_node = Mat2List(l_child_node)
		u_child_node = Mat2List(u_child_node)
		d_child_node = Mat2List(d_child_node)
		r_child_node = Mat2List(r_child_node)
        
        #Inititalizing the dictionary with spaces to append subsequent nodes
		path_track[str(a)] = []                            

        #Validating if the child nodes have been already visited, if not append to the open list and the visited list
		if l_child_node not in visited_nodes:
			visited_nodes.append(l_child_node)
			parent_node.append(l_child_node)
            #Updating the values for the keys which are the new child nodes in the dictionary
			path_track[str(a)].append(l_child_node)
		if u_child_node not in visited_nodes:
			visited_nodes.append(u_child_node)
			parent_node.append(u_child_node)
            #Updating the values for the keys which are the new child nodes in the dictionary
			path_track[str(a)].append(u_child_node)
		if r_child_node not in visited_nodes:
			visited_nodes.append(r_child_node)
            #Updating the values for the keys which are the new child nodes in the dictionary
			parent_node.append(r_child_node)
			path_track[str(a)].append(r_child_node)
		if d_child_node not in visited_nodes:
			visited_nodes.append(d_child_node)
			parent_node.append(d_child_node)
            #Updating the values for the keys which are the new child nodes in the dictionary
			path_track[str(a)].append(d_child_node)

        #Verifying if the goal state is reached
		for x in visited_nodes:
			if x == g:
				print(x)
				flag='F'
				break	
				
        #Break out of the loop if goal is reached
		if flag=='F':
			print('goal reached')
			break


#Backtracking to find the paths traversed from the initial state to the final state
final_state = g
val = g
goal = s
path_track_list=[]
while val!=goal:
    for key, values in path_track.items():
        while val in values:
            key= ast.literal_eval(key) #converting strings of lists to pure lists
            val = key
            path_track_list.append(val)
path_track_list=path_track_list[::-1]
path_track_list.append(final_state) 

# File nodePath.txt to write all the nodes traversed from start to goal
F = open('nodePath1.txt', 'w')
# List of numbers
for c in visited_nodes:
	for i in c:
		F.write(str(i)+' ')
	F.write('\n')
# Close the file
F.close()

path_track_list=path_track_list[::-1]
# File nodePath.txt to backtrack the paths followed from goal to start
F = open('nodetrack1.txt', 'w')
# List of numbers
for c in path_track_list:
	for i in c:
		F.write(str(i)+' ')
	F.write('\n')
# Close the file
F.close()

#Printing the path followed from finish to start
for i in path_track_list:
    print(PrintMatrix(i))
    print('\n')

#Printing the total time taken
print("total time:")
print(time.time()-start_time)	
