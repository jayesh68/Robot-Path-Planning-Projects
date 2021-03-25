#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 23 08:20:17 2021

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
from actionSet import actionSet


oblist1=[]       #List to store the obstacle coordinates for final animation
riglist=set([])
radius=10
clearance=5
dist=radius+clearance
xmax=400                    #Width of the map
ymax=300                    #Height of the map
#s = [1,1]                   #Start Position Test Case1
#g = [399,299]               #Goal Position Test Case1
solvable=False
visited_nodes = set([])     #Set consisting of all the nodes traversed by the point robot
visited=[]                  #Containing the list of visited nodes. Would be used for animating the visited states in the map
path_track={}               #Dictionary storing the parent nodes of the different child nodes to backtrack the path followed

l=0
q = PriorityQueue()         #Setting a priority queue
distance = {}               #Dictionary to store the distance of a node from the previous node 

act=actionSet()


def cost_update(child,loc,cost):
    #Checking if the child nodes are visited or not, if they lie within the resolution specified and if present in the obstacle space
    if (str(child) not in riglist) and (child[0]>0 and child[0]<xmax) and (child[1]>0 and child[1]<ymax) and (child is not None):
        if (str(child) in visited_nodes):
            new_cost=cost+distance[str(loc)]
            if new_cost < distance[str(child)]:   #If node already visited updating the node with the new cost if new cost is less than the original value
                distance[str(child)] = new_cost
                path_track[str(loc)].append(child)   #Updating the parent information
        else:
            visited_nodes.add(str(child))         #Adding the child nodes to the set of visited nodes
            visited.append(child)
            new_cost=cost+distance[str(loc)]      #Calculating the new cost
            distance[str(child)]=new_cost         #Setting the new cost of the node
            q.put([new_cost, child])              #Updating the priority queue
            path_track[str(loc)].append(child)   #Updating the parent information
            


def main():
    l=0
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

        #Inititalizing the dictionary to store information related to the parent node
        path_track[str(a[1])] = []

        #Getting the child nodes after moving in different positions
        l_child,cost1 = act.ActionMoveLeft(a[1],a[0])
        cost_update(l_child, a[1], cost1)
        u_child,cost2 = act.ActionMoveUp(a[1],a[0])
        cost_update(u_child, a[1], cost2)
        r_child,cost3= act.ActionMoveRight(a[1],a[0])
        cost_update(r_child, a[1], cost3)
        d_child,cost4 = act.ActionMoveDown(a[1],a[0])
        cost_update(d_child, a[1], cost4)
        ul_child,cost5 = act.ActionMoveUL(a[1],a[0])
        cost_update(ul_child, a[1], cost5)
        ur_child,cost6 = act.ActionMoveUR(a[1],a[0])
        cost_update(ur_child, a[1], cost6)
        dl_child,cost7 = act.ActionMoveDL(a[1],a[0])
        cost_update(dl_child, a[1], cost7)
        dr_child,cost8 = act.ActionMoveDR(a[1],a[0])
        cost_update(dr_child, a[1], cost8)
                    
               
def backtracking (start, goal):
    #Backtracking to find the paths traversed from the initial state to the final state
    final_state = g
    val = g
    goal = s
    path_track_list=[]
    #print('Parent track',path_track)
    path_track_list.append(final_state) 
    
    while val!=goal:
        for key, values in path_track.items():
            #print('key',key,values)
            while val in values:
                key= ast.literal_eval(key) #converting strings of lists to pure lists
                val = key
                path_track_list.append(val)
    return path_track_list

def visualization(path_track_list):
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
                i+=1                                                                                     #uncomment if not required
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

#### Code execution starts here #####
if __name__ == "__main__":
    
    oblist1, riglist=act.getobstaclespace()
    
    while True:
        x1=int(input('Enter x coordinate of start node: '))
        y1=int(input('Enter y coordinate of start node: '))

        s = [x1,y1]
        x2=int(input('Enter x coordinate of goal node: '))
        y2=int(input('Enter y coordinate of goal node: '))
        g = [x2,y2]                 #Goal Position Test Case2  
        
        if s == g:  #Checking if goal node is the same as the start node
            print('goal node equal to start node. Re enter your points again')
            continue
        
        elif str(s) in riglist or str(g) in riglist: #checking if the goal or start node is in the obstaclespace including radius and clearance
            print('Starting or goal node in obstacle space. Re enter the points again')
            continue
        
        elif (s[0] <0 or s[0]> xmax) or (s[1]<0 or s[1] > ymax) or (g[0] <0 or g[0]> xmax) or (g[1]<0 or g[1] > ymax): #Checking if the start and goal node is within the grid(400x300)
            print('start/goal < 0 or greater than grid size. Re enter the points again')
            
        else:
            break
    
    
    print(s)                    
    print(g)
    q.put([0, s])               #Initializing the queue with a cost of 0 and the start node
    visited_nodes.add(str(s)) #Adding the start node to the set of visited nodes
    visited.append(s)         #Appending the visited list

    #Initializing the cost of all the points to infinity
    for i in range(0, xmax):
        for j in range(0, ymax):
            distance[str([i, j])] = 99999999 
    distance[str(s)] = 0 
    
    start_time = time.time()    #Program start time
    main()   
    #Time to reach goal state
    print(time.time()-start_time)

    path_track_list = backtracking(s, g)
    #Printing the total time taken to reach goal state and backtrack
    print("total time:")
    print(time.time()-start_time)  

    visualization(path_track_list)
