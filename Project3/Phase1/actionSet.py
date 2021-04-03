#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 07:34:14 2021

@author: jayesh
"""

import numpy as np
import math 

class actionSet:
    
    #Function to traverse in the downward direction
    def ActionMoveDown(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_y = int(curr_node[1])
        new_node_y-=1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,1

    #Function to traverse in the upward direction
    def ActionMoveUp(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_y = int(curr_node[1])
        new_node_y+=1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,1

    #Function to traverse to the left
    def ActionMoveLeft(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x-=1
        new_node_y = int(curr_node[1])
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,1

    #Function to traverse to the right
    def ActionMoveRight(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x+=1
        new_node_y = int(curr_node[1])
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,1
    
    #Function to traverse in the upward left direction
    def ActionMoveUL(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x -=1
        new_node_y = int(curr_node[1])
        new_node_y+=1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,math.sqrt(2)
    
    #Function to traverse in the upward right direction
    def ActionMoveUR(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x+= 1
        new_node_y = int(curr_node[1])
        new_node_y+= 1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,math.sqrt(2)

    #Function to traverse in the downward left direction
    def ActionMoveDL(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x-=1
        new_node_y = int(curr_node[1])
        new_node_y-=1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,math.sqrt(2)

    #Function to traverse in the downward right direction
    def ActionMoveDR(self,curr_node,cost):
        new_node=[]
        new_node_x = int(curr_node[0])
        new_node_x+=1
        new_node_y = int(curr_node[1])
        new_node_y-=1
        new_node = [new_node_x,new_node_y]
        if new_node[0]>=0 and new_node[1]>=0:
            return new_node,math.sqrt(2)

    #Function run initially to set the obstacle coordinates in the image and append to a list
    def getobstaclespace(self):
        oblist=set([])   #Set to store the obstacle coordinates
        oblist1=[]       #List to store the obstacle coordinates for final animation
        riglist=set([])
        radius=10
        clearance=5
        dist= radius + clearance
        xmax=400
        ymax=300
        #print('ob space')
        for x in range(0,401):
            for y in range(0,301):
                
                #Rectangular Object
                if y-0.7*x>=74.28 and y-0.7*x <= 98.76 and y+1.425*x>=176.42 and y+1.428*x<=438.045:
                    oblist1.append([x,y])   
                    
                if y>=(x-44.316)*math.tan(35*math.pi/180)+87.109 and y <= (x-15.6376)*math.tan(35*math.pi/180)+128.066 and y>=-math.tan(55*math.pi/180)*(x-15.637)+128.066 and y<=-math.tan(55*math.pi/180)*(x-163.084)+231.31:
                        riglist.add(str([x,y]))

                #Circle Object
                if ((x-90)**2 + (y-70)**2)<(35**2):
                    oblist1.append([x,y])
                
                if ((x-90)**2 + (y-70)**2)<((35+dist)**2):
                    riglist.add(str([x,y]))
                
                #Ellipse Object
                if(((x-246)**2)/(60)**2 +((y-145)**2)/(30)**2 <= 1):
                    oblist1.append([x,y])
            
                if(((x-246)**2)/(60+dist)**2 +((y-145)**2)/(30+dist)**2 <= 1):
                    riglist.add(str([x,y]))
                
                #Polygon Shaped Object
                if (x>=200 and x<=230) and (y>=230 and y<=280):    
                    if (x>=200 and x<=210 and y>=240 and y<=270):
                        oblist1.append([x,y])
                    if (y>=270 and y<=280 and x>=210 and x<=230):
                        oblist1.append([x,y])
                    if (y>=230 and y<=240 and x>=210 and x<=240):
                        oblist1.append([x,y])
                    if (x<=210 and y<=240):
                        oblist1.append([x,y])
                    if (x<=210 and y>=270 and y<=280):
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
            
                #Resolution Check
                if x>=0 and x<=dist:
                    riglist.add(str([x,y]))
                
                if y>=0 and y<=dist:
                    riglist.add(str([x,y])) 
               
                if x>=(xmax-dist) and x<=xmax:
                    riglist.add(str([x,y]))
                
                if y>=(ymax-dist)and y<=ymax:
                    riglist.add(str([x,y]))
                        
        return oblist1,riglist