#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 14:06:18 2021

@author: jayesh
"""


import numpy as np
import copy
import math
import time
import ast
import cv2
import pygame
import os
from queue import PriorityQueue

#Creating an image to use for animation
img = np.zeros((301, 401), np.uint8)

oblist=set([])   #Set to store the obstacle coordinates
oblist1=[]       #List to store the obstacle coordinates for final animation
riglist=set([])
radius=10
clearance=5
dist=radius+clearance
xmax=400                    #Width of the map
ymax=300                    #Height of the map

#Function to traverse in the downward direction
def ActionMoveDown(curr_node,cost):
    #print('down')
    curr_node1 = copy.deepcopy(curr_node)
    #print('d',curr_node1)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_y = int(curr_node1[1])
    new_node_y-=1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,1
    else:
        return curr_node1,cost

#Function to traverse in the upward direction
def ActionMoveUp(curr_node,cost):
    #print('up')
    curr_node1 = copy.deepcopy(curr_node)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_y = int(curr_node1[1])
    new_node_y+=1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,1
    else:
        return curr_node1,cost

#Function to traverse to the left
def ActionMoveLeft(curr_node,cost):
    #print('currleft',curr_node)
    curr_node1 = copy.deepcopy(curr_node)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    #print('lcurr',curr_node1[0],curr_node1[1],len(curr_node1))
    new_node_x = int(curr_node1[0])
    new_node_x-=1
    new_node_y = int(curr_node1[1])
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,1
    else:
        return curr_node1,cost

#Function to traverse to the right
def ActionMoveRight(curr_node,cost):
    #print('right')
    curr_node1 = copy.deepcopy(curr_node)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_x+=1
    new_node_y = int(curr_node1[1])
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,1
    else:
        return curr_node1,cost

#Function to traverse in the upward left direction
def ActionMoveUL(curr_node,cost):
    #print('ul')
    curr_node1 = copy.deepcopy(curr_node)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_x -=1
    new_node_y = int(curr_node1[1])
    new_node_y+=1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,math.sqrt(2)
    else:
        return curr_node1,cost

#Function to traverse in the upward right direction
def ActionMoveUR(curr_node,cost):
    #print('ur')
    curr_node1 = copy.deepcopy(curr_node)
    x = str(curr_node1[0])
    y = str(curr_node1[1])
    curr_node1 = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_x+= 1
    new_node_y = int(curr_node1[1])
    new_node_y+= 1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,math.sqrt(2)
    else:
        return curr_node1,cost

#Function to traverse in the downward left direction
def ActionMoveDL(curr_node,cost):
    #print('dl')
    curr_node1 = copy.deepcopy(curr_node)
    x = curr_node1[0]
    y = curr_node1[1]
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_x-=1
    new_node_y = int(curr_node1[1])
    new_node_y-=1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,math.sqrt(2)
    else:
        return curr_node1,cost

#Function to traverse in the downward right direction
def ActionMoveDR(curr_node,cost):
    #print('dr')
    curr_node1 = copy.deepcopy(curr_node)
    x = str(curr_node1[0])
    y = str(curr_node1[1])
    curr_node = (x,y)
    new_node=()
    new_node_x = int(curr_node1[0])
    new_node_x+=1
    new_node_y = int(curr_node1[1])
    new_node_y-=1
    new_node = [new_node_x,new_node_y]
    if new_node[0]>=0 and new_node[1]>=0:
        return new_node,math.sqrt(2)
    else:
        return curr_node1,1

#Function run initially to set the obstacle coordinates in the image and append to a list
def getobstaclespace():
    #print('ob space')
    for x in range(0,401):
        for y in range(0,301):
            
            #Rectangular Object
            if y-0.7*x>=74.28 and y-0.7*x <= 98.76 and y+1.425*x>=176.42 and y+1.428*x<=438.045:
                img[y][x]=255
                oblist.add(str([x,y]))
                oblist1.append([x,y])
            '''
            if y-0.7*x>=(74.28-dist) and y-0.7*x <= (98.76+dist) and y+1.425*x>=(176.42-dist) and y+1.428*x<=(438.045+dist):
                riglist.add(str([x,y]))
            
            if y-0.7*x>=(59.28) and y-0.7*x <= (113.76) and y+1.425*x>=(161.42) and y+1.428*x<=(453.04):
                riglist.add(str([x,y]))
            '''    

            if y>=(x-44.316)*math.tan(35*math.pi/180)+87.109 and y <= (x-15.6376)*math.tan(35*math.pi/180)+128.066 and y>=-math.tan(55*math.pi/180)*(x-15.637)+128.066 and y<=-math.tan(55*math.pi/180)*(x-163.084)+231.31:
                riglist.add(str([x,y]))

            #Circle Object
            if ((x-90)**2 + (y-70)**2)<(35**2):
                img[y][x]=255
                oblist.add(str([x,y]))
                oblist1.append([x,y])
                
            if ((x-90)**2 + (y-70)**2)<((35+dist)**2):
                riglist.add(str([x,y]))
                
            #Ellipse Object
            if(((x-246)**2)/(60)**2 +((y-145)**2)/(30)**2 <= 1):
                img[y][x]=255
                oblist.add(str([x,y]))
                oblist1.append([x,y])
            
            if(((x-246)**2)/(60+dist)**2 +((y-145)**2)/(30+dist)**2 <= 1):
                riglist.add(str([x,y]))
                
            #Polygon Shaped Object
            if (x>=200 and x<=230) and (y>=230 and y<=280):    
                if (x>=200 and x<=210 and y>=240 and y<=270):
                    img[y][x]=255
                    oblist.add(str([x,y]))
                    oblist1.append([x,y])
                if (y>=270 and y<=280 and x>=210 and x<=230):
                    img[y][x]=255
                    oblist.add(str([x,y]))
                    oblist1.append([x,y])
                if (y>=230 and y<=240 and x>=210 and x<=240):
                    img[y][x]=255
                    oblist.add(str([x,y]))
                    oblist1.append([x,y])
                if (x<=210 and y<=240):
                    img[y][x]=255
                    oblist.add(str([x,y]))
                    oblist1.append([x,y])
                if (x<=210 and y>=270 and y<=280):
                    img[y][x]=255
                    oblist.add(str([x,y]))
                    oblist1.append([x,y])
                    
            if (x>=200-dist and x<=230+dist) and (y>=230-dist and y<=280+dist):    
                if (x>=200-dist and x<=210+dist and y>=240-dist and y<=270+dist):
                    riglist.add(str([x,y]))
                if (y>=270-dist and y<=280+dist and x>=210-dist and x<=230+dist):
                    riglist.add(str([x,y]))
                if (y>=230-dist and y<=240+dist and x>=210-dist and x<=240+dist):
                    riglist.add(str([x,y]))
                if (x<=210+dist and y<=240+dist):
                    riglist.add(str([x,y]))
                if (x<=210+dist and y>=270-dist and y<=280+dist):
                    riglist.add(str([x,y]))
            
            if x>=0 and x<=dist:
                riglist.add(str([x,y]))
                
            if y>=0 and y<=dist:
                riglist.add(str([x,y])) 
               
            if x>=(xmax-dist) and x<=xmax:
                riglist.add(str([x,y]))
                
            if y>=(ymax-dist)and y<=ymax:
                riglist.add(str([x,y]))
            
                      
start_time = time.time()    #Program start time
getobstaclespace()
#s = [1,1]                   #Start Position Test Case1
#g = [399,299]               #Goal Position Test Case1
xmax=400                    #Width of the map
ymax=300                    #Height of the map
solvable=True

while(1):
    x1=int(input('Enter x coordinate of start node'))
    y1=int(input('Enter y coordinate of start node'))

    s = [x1,y1]
    x2=int(input('Enter x coordinate of goal node'))
    y2=int(input('Enter y coordinate of goal node'))
    g = [x2,y2]                 #Goal Position Test Case2  
    
    if s == g:  #Checking if goal node is the same as the start node
        print('goal node equal to start node. Re enter your points again')
        continue
    
    elif str(s) in oblist or str(g) in oblist: #checking if the goal or start node is in the obstacspace
        print('Starting or goal node in obstacle space. Re enter the points again')
        solvable=False
        continue
    
    elif (s[0] <0 or s[0]> xmax) or (s[1]<0 or s[1] > ymax) or (g[0] <0 or g[0]> xmax) or (g[1]<0 or g[1] > ymax): #Checking if the start and goal node is within the grid(400x300)
        print('start/goal < 0 or greater than grid size. Re enter the points again')
        
    else:
        break

visited_nodes = set([])     #Set consisting of all the nodes traversed by the point robot
visited=[]                  #Containing the list of visited nodes. Would be used for animating the visited states in the map
child_node = []             #stores the child states after point robot moves to different positions
path_track={}               #Dictionary storing the parent nodes of the different child nodes to backtrack the path followed
print(s)                    
print(g)
l=0
q = PriorityQueue()         #Setting a priority queue
q.put([0, s])               #Initializing the queue with a cost of 0 and the start node
distance = {}               #Dictionary to store the distance of a node from the previous node 

#Initializing the cost of all the points to infinity
for i in range(0, xmax):
    for j in range(0, ymax):
        distance[str([i, j])] = 99999999 

#Setting the cost of the start node to 0
distance[str(s)] = 0 
visited_nodes.add(str(s)) #Adding the start node to the set of visited nodes
visited.append(s)         #Appending the visited list

if solvable:
    #print('solvable')
    while not q.empty():  #Process when queue is not empty
        print(l)
        a=q.get()         #Varibale to store the cost and node position
        #print('queue',a[0],a[1],len(a))
        
        #Checking if goal is reached or not
        if a[1]==g:
            print('goal reached')
            break

        l+=1
        
        visited.append(a[1])
        #Getting the child nodes after moving in different positions
        l_child,cost1 = ActionMoveLeft(a[1],a[0])
        u_child,cost2 = ActionMoveUp(a[1],a[0])
        r_child,cost3= ActionMoveRight(a[1],a[0])
        d_child,cost4 = ActionMoveDown(a[1],a[0])
        ul_child,cost5 = ActionMoveUL(a[1],a[0])
        ur_child,cost6 = ActionMoveUR(a[1],a[0])
        dl_child,cost7 = ActionMoveDL(a[1],a[0])
        dr_child,cost8 = ActionMoveDR(a[1],a[0])
     
        #Inititalizing the dictionary to store information related to the parent node
        path_track[str(a[1])] = []        
                  
        #Checking if the child nodes are visited or not, if they lie within the resolution specified and if present in the obstacle space
        if (str(l_child) not in riglist) and (str(l_child) in visited_nodes) and (l_child[0]>0 and l_child[0]<xmax) and (l_child[1]>0 and l_child[1]<ymax):
            new_cost=cost1+distance[str(a[1])]
            if new_cost < distance[str(l_child)]:   #If node already visited updating the node with the new cost if new cost is less than the original value
                distance[str(l_child)] = new_cost         
        if (str(l_child) not in riglist) and (str(l_child) not in visited_nodes) and (l_child[0]>0 and l_child[0]<xmax) and (l_child[1]>0 and l_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(l_child))         #Adding the child nodes to the set of visited nodes
            visited.append(l_child)
            new_cost=cost1+distance[str(a[1])]      #Calculating the new cost
            distance[str(l_child)]=new_cost         #Setting the new cost of the node
            path_track[str(a[1])].append(l_child)   #Updating the parent information
            
        if (str(r_child) not in riglist) and (str(r_child) in visited_nodes) and (r_child[0]>0 and r_child[0]<xmax) and (r_child[1]>0 and r_child[1]<ymax):
            new_cost=cost2+distance[str(a[1])]
            if new_cost < distance[str(r_child)]:
                distance[str(r_child)] = new_cost      
        #Similarly perform for the remaining nodes in different directions
        if (str(r_child) not in riglist) and (str(r_child) not in visited_nodes) and (r_child[0]>0 and r_child[0]<xmax) and (r_child[1]>0 and r_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(r_child))
            visited.append(r_child)
            new_cost=cost2+distance[str(a[1])]
            distance[str(r_child)]=new_cost
            q.put([new_cost, r_child])
            #print('rchild',r_child)
            path_track[str(a[1])].append(r_child)
             
        if (str(u_child) not in riglist) and (str(u_child) in visited_nodes) and (u_child[0]>0 and u_child[0]<xmax) and (u_child[1]>0 and u_child[1]<ymax):
            new_cost=cost3+distance[str(a[1])]
            if new_cost < distance[str(u_child)]:
                distance[str(u_child)] = new_cost      
        if (str(u_child) not in riglist) and (str(u_child) not in visited_nodes) and (u_child[0]>0 and u_child[0]<xmax) and (u_child[1]>0 and u_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(u_child))
            visited.append(u_child)
            new_cost=cost3+distance[str(a[1])]
            distance[str(u_child)]=new_cost
            q.put([new_cost, u_child])
            path_track[str(a[1])].append(u_child)
                         
        if (str(d_child) not in riglist) and (str(d_child) in visited_nodes) and (d_child[0]>0 and d_child[0]<xmax) and (d_child[1]>0 and d_child[1]<ymax):
            new_cost=cost4+distance[str(a[1])]
            if new_cost < distance[str(d_child)]:
                distance[str(d_child)] = new_cost      
        if (str(d_child) not in riglist) and (str(d_child) not in visited_nodes) and (d_child[0]>0 and d_child[0]<xmax) and (d_child[1]>0 and d_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(d_child))
            visited.append(d_child)
            new_cost=cost4+distance[str(a[1])]
            distance[str(d_child)]=new_cost
            q.put([new_cost, d_child])
            path_track[str(a[1])].append(d_child)
               
        if (str(ul_child) not in riglist) and (str(ul_child) in visited_nodes) and (ul_child[0]>0 and ul_child[0]<xmax) and (ul_child[1]>0 and ul_child[1]<ymax):
            new_cost=cost5+distance[str(a[1])]
            if new_cost < distance[str(ul_child)]:
                distance[str(ul_child)] = new_cost    
        if (str(ul_child) not in riglist) and (str(ul_child) not in visited_nodes) and (ul_child[0]>0 and ul_child[0]<xmax) and (ul_child[1]>0 and ul_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(ul_child))
            visited.append(ul_child)
            new_cost=cost5+distance[str(a[1])]
            distance[str(ul_child)]=new_cost
            q.put([new_cost, ul_child])
            path_track[str(a[1])].append(ul_child)
            
        if (str(dl_child) not in riglist) and (str(dl_child) in visited_nodes) and (dl_child[0]>0 and dl_child[0]<xmax) and (dl_child[1]>0 and dl_child[1]<ymax):
            new_cost=cost7+distance[str(a[1])]
            if new_cost < distance[str(dl_child)]:
                distance[str(dl_child)] = new_cost       
        if (str(dl_child) not in riglist) and (str(dl_child) not in visited_nodes) and (dl_child[0]>0 and dl_child[0]<xmax) and (dl_child[1]>0 and dl_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(dl_child))
            visited.append(dl_child)
            new_cost=cost7+distance[str(a[1])]
            distance[str(dl_child)]=new_cost
            q.put([new_cost, dl_child])
            path_track[str(a[1])].append(dl_child)
            
        if (str(ur_child) not in riglist) and (str(ur_child) in visited_nodes) and (ur_child[0]>0 and ur_child[0]<xmax) and (ur_child[1]>0 and ur_child[1]<ymax):
            new_cost=cost6+distance[str(a[1])]
            if new_cost < distance[str(ur_child)]:
                distance[str(ur_child)] = new_cost     
        if (str(ur_child) not in riglist) and (str(ur_child) not in visited_nodes) and (ur_child[0]>0 and ur_child[0]<xmax) and (ur_child[1]>0 and ur_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(ur_child))
            visited.append(ur_child)
            new_cost=cost6+distance[str(a[1])]
            distance[str(ur_child)]=new_cost
            q.put([new_cost, ur_child])
            path_track[str(a[1])].append(ur_child)
            
        if (str(dr_child) not in riglist) and (str(dr_child) in visited_nodes) and (dr_child[0]>0 and dr_child[0]<xmax) and (dr_child[1]>0 and dr_child[1]<ymax):
            new_cost=cost8+distance[str(a[1])]
            if new_cost < distance[str(dr_child)]:
                distance[str(dr_child)] = new_cost     
        if (str(dr_child) not in riglist) and (str(dr_child) not in visited_nodes) and (dr_child[0]>0 and dr_child[0]<xmax) and (dr_child[1]>0 and dr_child[1]<ymax):
            #print('l',l_child)
            visited_nodes.add(str(dr_child))
            visited.append(dr_child)
            new_cost=cost8+distance[str(a[1])]
            distance[str(dr_child)]=new_cost
            q.put([new_cost, dr_child])
            path_track[str(a[1])].append(dr_child)
            

        #cv2.imshow('maze',img)
        #cv2.waitKey(1)
        
#Time to reach goal state
print(time.time()-start_time)

#Backtracking to find the paths traversed from the initial state to the final state
final_state = g
val = g
goal = s
path_track_list=[]
#print('Parent track',path_track)
while val!=goal:
    for key, values in path_track.items():
        #print('key',key,values)
        while val in values:
            key= ast.literal_eval(key) #converting strings of lists to pure lists
            val = key
            path_track_list.append(val)
path_track_list=path_track_list[::-1]
path_track_list.append(final_state) 

# File nodePath.txt to write all the nodes traversed from start to goal
F = open('nodePath11.txt', 'w')
# List of numbers
for c in visited:
    for i in c:
        F.write(str(i)+' ')
    F.write('\n')
# Close the file
F.close()

path_track_list=path_track_list[::-1]
# File nodePath.txt to backtrack the paths followed from goal to start
F = open('nodetrack11.txt', 'w')
# List of numbers
for c in path_track_list:
    for i in c:
        F.write(str(i)+' ')
    F.write('\n')
# Close the file
F.close()

#Printing the total time taken to reach goal state and backtrack
print("total time:")
print(time.time()-start_time)  

#Creating an animation using pygame
pygame.init()

display_width = 400
display_height = 300

gameDisplay = pygame.display.set_mode((display_width,display_height),pygame.SCALED)
pygame.display.set_caption('Djikstra Animation')

black = (0,0,0)         #Color represnting the background of image
white = (0,255,255)     #Color respresenting the visited nodes
yellow=(255,255,0)      #Color representing the obstacles

i=0
#surf = pygame.surfarray.make_surface(img)

clock = pygame.time.Clock()
done = False
while not done:
    for event in pygame.event.get():   
        if event.type == pygame.QUIT:  
            done = True   
       
    gameDisplay.fill(black)

    #Setting the obstacle space in the animation
    for path in oblist1:
            x = int(path[0])
            y = abs(300-int(path[1]))
            pygame.draw.rect(gameDisplay, yellow, [x,y,1,1])
      
    #print('Visited',visited)
    #Visualizing the visited states in the animation
    for path in visited:    
            x =path[0]
            y = abs(300-path[1])
            pygame.display.flip()
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            #pygame.image.save(gameDisplay, f"/home/jayesh/Documents/ENPM661_PROJECT1/map1/{i}.png")  #Saving the images to create a video 
            #i+=1                                                                                     #uncomment if not required
            pygame.time.wait(0)                    
    
    #Visualizing the path taken from start to node
    for path in path_track_list:
        pygame.time.wait(10)
        #time.sleep(0.00005)
        x = path[0]
        y = abs(300-path[1])
        pygame.display.flip()
        pygame.draw.rect(gameDisplay, (255,5,5), [x,y,1,1])
        #pygame.image.save(gameDisplay, f"/home/jayesh/Documents/ENPM661_PROJECT1/map1/{i}.png")         #Saving the images to create a video
        i+=1                                                                                            #uncomment if not required
        pygame.time.wait(10)
   
    done = True

pygame.quit()

#Writing to video. Uncomment if required
'''
size=(400,300)
out = cv2.VideoWriter('p2dijkstra.avi',cv2.VideoWriter_fourcc(*'DIVX'), 800, size)
file_list=os.listdir('/home/jayesh/Documents/ENPM661_PROJECT1/map1')

new_list=[]

for file in file_list:
    #print(file)
    a=file.split('.')[0]
    #print(a)
    new_list.append(a)
      
#print(new_list)

for i in range(0,len(new_list)):
    filename=f'/home/jayesh/Documents/ENPM661_PROJECT1/map1/{i}.png'
    #print(filename)
    j+=1 
    #print(filename)
    img = cv2.imread(filename)
    out.write(img)
#cv2.imshow('obstacle',img)

out.release()

cv2.waitKey(0)
cv2.destroyAllWindows()
'''