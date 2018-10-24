'''
Author:  Anton Medvdev, amedvedev2013@my.fit.edu
Course:  CSE 5280, Spring 2017
Project: Proj 02, Animation
'''

#Imports
from visual import *
import numpy as np
import math

#Constants
#############################

#relaxation time
relaxation = 1.0   						

#desigreen velocity
velocity = vector(2.0,2.0,0)          	

#repulsive factor
repV = 1.5
sigma = 2.5

#obstacle factor
obs = 2   									
Radius = 1.5   				

#stept ime
dt = .03
#############################


#List of objects 
#############################

#Blue group
groupBlue = list()
groupBlue.append(sphere(pos=vector(-20,2), radius=1, color=color.blue))
groupBlue.append(sphere(pos=vector(-20,-2), radius=1, color=color.blue))
groupBlue.append(sphere(pos=vector(-16,2), radius=1, color=color.blue))
groupBlue.append(sphere(pos=vector(-16,-2), radius=1, color=color.blue))
for a in groupBlue:
    a.velocity = velocity
    
#Green Group
groupGreen = list()
groupGreen.append(sphere(pos=vector(20,2), radius=1, color=color.green))
groupGreen.append(sphere(pos=vector(20,-2), radius=1, color=color.green))
groupGreen.append(sphere(pos=vector(16,2), radius=1, color=color.green))
groupGreen.append(sphere(pos=vector(16,-2), radius=1, color=color.green))
for a in groupGreen:
    a.velocity = velocity 

#Obstacles
obstacles = list()
obstacles.append(cylinder(pos=vector(-2,0),axis=(0,2,1), radius=Radius))
#obstacles.append(cylinder(pos=vector(4,-4),axis=(0,0,1), radius=1.75))

#Goal Positions
finishBlue = list()
finishGreen = list()

finishBlue.append(vector(16,2))
finishBlue.append(vector(16,-2))
finishBlue.append(vector(20,2))
finishBlue.append(vector(20,-2))

finishGreen = list()
finishGreen.append(vector(-16,2))
finishGreen.append(vector(-16,-2))
finishGreen.append(vector(-20,2))
finishGreen.append(vector(-20,-2))

#############################


#Extra methods
#############################

#summation of repulsive pedestrian forces
def repGr(a, group, finish):

    repForce = 0
    
    for b in group:
    
        finishB = finish[group.index(b)]
        directionB = (finishB - b.pos)/(np.linalg.norm(finishB - b.pos))
        ellipseB = math.sqrt((math.pow((np.linalg.norm(a.pos-b.pos) + np.linalg.norm((a.pos-b.pos)-(dt*directionB))),2) + math.pow(dt,2)))/2
        repForce += np.multiply((-np.gradient(a.pos-b.pos)),(repV*math.pow(math.e,-ellipseB/sigma)))
        
    return repForce
    
    
#summation of repulsive obstacle forces
def obsGr(a, group):
    totalObs = 0
    for b in group:
        totalObs += np.multiply((-np.gradient(a.pos-b.pos)),(obs*math.pow(math.e,-(np.linalg.norm(a.pos-b.pos)/Radius))))
    return totalObs    

#############################


#simulation
#############################
while 1:
    rate(500)
    
    for a in groupBlue:
    
    	#get finish position of the object 
        finish = finishBlue[groupBlue.index(a)]
        
        if not np.linalg.norm(a.pos - finish) <= .5:
        
        	#get direction of the object
        	DesiredDirection = (finish - a.pos)/(np.linalg.norm(finish - a.pos))
        	
        	#get desired acceleration
        	desVelocity = np.multiply(velocity,DesiredDirection)
        	acceleration = (1/relaxation)*(desVelocity - a.velocity)
        	
        	#repulsive force for the blue group
        	repF = repGr(a,groupBlue,finishBlue)
        	
        	#repulsive force for the obstacles
        	obstical = obsGr(a,obstacles)
        	a.velocity = a.velocity + acceleration + repF + obstical
        	
        	#movement
        	a.pos = a.pos + a.velocity*dt
        	
    for a in groupGreen:
    
    	#get finish position of the object 
    	finish = finishGreen[groupGreen.index(a)]
    	
    	if not np.linalg.norm(a.pos - finish) <= .5:
    	
    		#get direction of the object
        	DesiredDirection = (finish - a.pos)/(np.linalg.norm(finish - a.pos))
        	
        	#get desired acceleration
        	desVelocity = np.multiply(velocity,DesiredDirection)
        	acceleration = (1/relaxation)*(desVelocity - a.velocity)
        	
        	#repulsive force for the blue group
        	repF = repGr(a,groupGreen,finishGreen)
        	
        	#repulsive force for the obstacles
        	obstical = obsGr(a,obstacles)
        	a.velocity = a.velocity + acceleration + repF + obstical
        	
        	#movement
        	a.pos = a.pos + a.velocity*dt
        	
#############################








