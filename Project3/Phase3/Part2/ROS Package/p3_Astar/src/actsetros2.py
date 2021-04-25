#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  9 19:51:43 2021

@author: jayesh
"""


import numpy as np
import math 
import matplotlib.pyplot as plt
class actionSet:
    
    #Function to traverse in the downward direction 
    #Degree = 30
    def Action(self,curr_node,ul,ur):
        x = curr_node[0]
        y = curr_node[1]
        ang=curr_node[2]
        #print('before act',x,y,ang)
        t = 0
        r = 0.038
        l = 0.354
        #print('rpms',ul,ur)
        dt=0.1
        cost=0
        while t < 1:
            t = t + dt
            dx = 0.5*r * (ul + ur) * math.cos(ang*math.pi/180) * dt
            dy = 0.5*r * (ul + ur) * math.sin(ang*math.pi/180) * dt
            dtheta = (r / l) * (ur - ul) * dt
            #print('dt',dtheta)
            dtheta=dtheta*180/math.pi
            ang+= dtheta
            #print('dx','dy',dx,dy)
            #print('ang',ang)
            cost=cost+ math.sqrt(math.pow((0.5*r * (ul + ur) * math.cos(ang*math.pi/180) * dt),2)+math.pow((0.5*r * (ul + ur) * math.sin(ang*math.pi/180) * dt),2))
            x+= dx
            y+= dy
            #plt.plot([x, dx], [y, dy], color="blue")
            #plt.show()
            #print('adding',x,y)
            
        if ang >= 360 or ang<0:
            ang=ang%360
           
        #ang=180*ang/3.14
        #x=int(round(x/2)*2)
        #y=int(round(y/2)*2)
        #print('act',x,y,ang)
        #ang=int((round(ang/15)*15)//15)
        new_node = [x,y,int(ang)]        
        print('new node',new_node)
        return new_node,cost
    
    #Function run initially to set the obstacle coordinates in the image and append to a list
    def getobstaclespace(self):
        oblist1=[]       #List to store the obstacle coordinates for final animation
        riglist=set([])
        radius=0.35
        clearance=0.05
        dist= radius + clearance
        #xmax=510
        #ymax=510
        #print('ob space')
        
        for x in range(0,11):
            for y in range(0,11):
                
                if (x-2.00)**2 + (y-2.00)**2 <= 1.00**2:
                    oblist1.append([x,y])   
                    
                if (x-2.00)**2 + (y-8.00)**2 <= 1.00**2:
                    oblist1.append([x,y])
 
                # left square
                if 0.75 <= x <= 1.75:
                    if 4.25 <= y <= 5.75:
                        oblist1.append((x, y))
    
                # right square
                if 3.75 <= x <= 6.25:
                    if 4.25 <= y <= 5.75:
                        oblist1.append((x, y))
    
                # top left square
                if 7.25 <= x <= 8.75:
                    if 2.00 <= y <= 4.00:
                        oblist1.append((x, y))
                
                if (x-2.00)**2 + (y-2.00)**2 <= (1.00+dist)**2:
                    riglist.add(str([x,y])) 
                    
                if (x-2.00)**2 + (y-8.00)**2 <= (1.00+dist)**2:
                    riglist.add(str([x,y]))

    
                # left square
                if (0.75-dist) <= x <= (1.75+dist):
                    if (4.25-dist) <= y <= (5.75+dist):
                        riglist.add(str([x,y]))
    
                # right square
                if (3.75-dist) <= x <= (6.25+dist):
                    if (4.25-dist) <= y <= (5.75+dist):
                        riglist.add(str([x,y]))
    
                # top left square
                if (7.25-dist) <= x <= (8.75+dist):
                    if (2.00-dist) <= y <= (4.00+dist):
                        riglist.add(str([x,y]))
        
        #print('riglist',riglist)
        return oblist1,riglist