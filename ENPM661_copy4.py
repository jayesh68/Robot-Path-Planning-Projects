import numpy as np
import copy
import time
import ast

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


def BlankTileLocate(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	for i in range(len(curr_node1)):
		for j in range(len(curr_node1[i])):
			if curr_node1[i][j] == 0:
				row = i
				col = j
	return([row, col])


def ActMoveLeft(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
	# print('move left')
	if c != 0:
		curr_node1[r][c-1], curr_node1[r][c] = curr_node1[r][c], curr_node1[r][c-1]
		return(curr_node1, True)
	else:
		# print('left false')
		return(curr_node1, False)


def ActMoveRight(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node)[1]
	# print('move right')
	if c != 3:
		curr_node1[r][c+1], curr_node1[r][c] = curr_node1[r][c], curr_node1[r][c+1]
		# print('right true')
		return(curr_node1, True)
	else:
		# print('right false')
		return(curr_node1, False)


def ActMoveUp(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
	if r != 0:
		curr_node1[r-1][c], curr_node1[r][c] = curr_node1[r][c], curr_node1[r-1][c]
		return(curr_node1, True)
	else:
		# print('up false')
		return(curr_node1, False)


def ActMoveDown(curr_node):
	curr_node1 = copy.deepcopy(curr_node)
	r = BlankTileLocate(curr_node1)[0]
	c = BlankTileLocate(curr_node1)[1]
	# print('move down')
	if r != 3:
		# print('down true')
		curr_node1[r+1][c], curr_node1[r][c] = curr_node1[r][c], curr_node1[r+1][c]
		return(curr_node1, True)
	else:
		# print('down false')
		return(curr_node1, False)


def solvable(node):
	s = np.reshape(node, 16)
	inv_count = 0
	for i in range(len(s)):
		for j in range(i+1, len(s)):
			if s[i] > 0 and s[j] > 0 and s[i] > s[j]:
				inv_count += 1

	print('invcount', inv_count)
	if inv_count % 2 == 1:
		print('solvable')
		return True
	else:
		return False


def Mat2List(m):
    p = []
    for i in m.tolist():
            p = p + i
    return p

#s = [1, 2, 3, 4, 5, 6, 0, 8, 9, 10, 7, 12, 13, 14, 11, 15]
#s=[1,0,3,4,5,2,7,8,9,6,10,11,13,14,15,12]
#s=[0,2,3,4,1,5,7,8,9,6,11,12,13,10,14,15]
#s=[5,1,2,3,0,6,7,4,9,10,11,8,13,14,15,12]
s=[1,6,2,3,9,5,7,4,0,10,11,8,13,14,15,12]
#s=[9,2,5,4,13,11,0,3,15,1,7,8,10,6,14,12]
g = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0]
start_time = time.time()
visited_nodes = []
parent_node = []
parent_node.append(s)
status = solvable(s)
child_node = []
path_track={}
print(s)
print(g)
j = 0
flag = 'n'
status = True

if status == True:
	visited_nodes.append(s)
	if s == g:
		print('goal already reached')
	else:
		count = 1
		count2 = 0
		# print(parent_node)
		# parent_node.append([])
		# print('parent',parent_node)
		# while(len(parent_node)>0):
		# i=parent_node
		# parent_node.pop()
		# print('popping',parent_node)
		#print('count is', count)
		j = 1
		for a in parent_node:
			#print('length', len(parent_node))
			# print('i',i)
			#print('a', a)
			#print('loop:', j)
			#print('parent node', parent_node)
			l_child_node, statl = ActMoveLeft(PrintMatrix(a))
			u_child_node, statu = ActMoveUp(PrintMatrix(a))
			r_child_node, statr = ActMoveRight(PrintMatrix(a))
			d_child_node, statd = ActMoveDown(PrintMatrix(a))
			# print('visited',visited_nodes)
			# print('left_node',l_child_node)
			# print('upper_node',u_child_node)
			# print('right_node',r_child_node)
			# print('down_node',d_child_node)
			# if statl== True or statl== True or statl== True or statl== True:
			#	parent_node.clear()
			l_child_node = Mat2List(l_child_node)
			u_child_node = Mat2List(u_child_node)
			d_child_node = Mat2List(d_child_node)
			r_child_node = Mat2List(r_child_node)
			# print(parent_node)
			path_track[str(a)] = []
			if l_child_node not in visited_nodes: 
			    path_track[str(a)].append(l_child_node)
			#if l_copy!=u_copy:
			#    path_track[str(u_copy)] = []
			if u_child_node not in visited_nodes:
			    path_track[str(a)].append(u_child_node)
			#if u_copy!=r_copy:
			#    paid_track[str(r_copy)] = []
			if r_child_node not in visited_nodes:
			    path_track[str(a)].append(r_child_node)
			#if r_copy!=d_copy: 
			#    paid_track[str(d_copy)] = []
			if d_child_node not in visited_nodes: 
			    path_track[str(a)].append(d_child_node)
                
			# print('left child',l_child_node)
			if l_child_node not in visited_nodes:
			# print(l_child_node)
				visited_nodes.append(l_child_node)
			# print(parent_node[1])
				parent_node.append(l_child_node)

			if u_child_node not in visited_nodes:
				# print(u_child_node)
				visited_nodes.append(u_child_node)
				parent_node.append(u_child_node)
			if r_child_node not in visited_nodes:
				# print(r_child_node)
				visited_nodes.append(r_child_node)
				parent_node.append(r_child_node)

			if d_child_node not in visited_nodes:
				visited_nodes.append(d_child_node)
				# print(d_child_node)
				parent_node.append(d_child_node)

			# print('parent1',parent_node[0])

			for x in visited_nodes:
			# print('goal',g)
				# print('set of nodes traversed',x)
				if x == g:
					print(x)
					#print('visited',visited_nodes)                                            
					#print('len of visited states', len(visited_nodes))
					flag='F'
					break	
				
			if flag=='F':
				print('goal reached')
				break
			j+=1
			
else:
	print('unsolvable')

#Backtracking
final_state = g
val = g
goal = s
path_track_list=[]
while val!=goal:
    for key, values in path_track.items():
        #print('key',key,'val',values)
        while val in values:
            #print('values',val)
            key= ast.literal_eval(key)
            val = key
            path_track_list.append(val)
#print('path track 0',path_track_list)
path_track_list=path_track_list[::-1]
path_track_list.append(final_state) 
#print('path track 01',path_track_list)

# Open the file for writing nodePath.txt 
F = open('nodePath.txt', 'w')
# List of numbers
for c in visited_nodes:
	for i in c:
		F.write(str(i)+' ')
	F.write('\n')
# Close the file
F.close()
#print('Dictionary',path_track)
#print('Path Track List before',path_track_list)
path_track_list=path_track_list[::-1]
#print('Path Track List',path_track_list)
for i in path_track_list:
    print(PrintMatrix(i))
    print('\n')
print('START')

print("total time:")
print(time.time()-start_time)	
