#!/usr/bin/env python

import numpy as np
from matplotlib import path
import pdb



#Helper functions
def isPointInside(xint,myline):
    return (xint >= myline[0,0] and xint <= myline[1,0]) or \
           (xint >= myline[1,0] and xint <= myline[0,0])

def slope(line):
    return np.float64(line[1,1] - line[0,1])/(line[1,0] - line[0,0])

def intercept(line, m):
    return line[0,1] - m*line[0,0]

def check_intercept( line1, line2 ):

    m1 = slope(line1)
    m2 = slope(line2)

    if m1 == m2:
        return 0

    if m1 == np.inf or m1 == -np.inf:
        line1[0,0] = line1[0,0] + .001
        m1 = slope(line1)

    if m2 == np.inf or m2 == -np.inf:
        line2[0,0] = line2[0,0] + .001
        m2 = slope(line2)

    # if m1 > 10000 or m1 < -10000 or m1 == np.nan:
    #   line1[0,0] = line1[0,0] + .001
    #   m1 = slope(line1)

    # if m2 > 10000 or m2 < -10000 or m2 == np.nan:
    #   line2[0,0] = line2[0,0] + .001
    #   m2 = slope(line2)

    b1 = intercept(line1,m1)
    b2 = intercept(line2,m2)

    xintersect = (b2-b1)/(m1-m2)
    yintersect = m1*xintersect + b1

    return isPointInside(xintersect,line1) and isPointInside(xintersect,line2)

def check_point(p1, p2):
    return ((p1[0] == p2[0]) and (p1[1] == p2[1]))

def check_endpoints(line1, line2):
    return check_point(line1[0,:], line2[0,:]) + \
           check_point(line1[0,:], line2[1,:]) + \
           check_point(line1[1,:], line2[0,:]) + \
           check_point(line1[1,:], line2[1,:])



def vgraph( obstacles, vertices , plotter=None):

    graph = np.zeros((len(vertices), len(vertices)))

    lines = []

    for i in range(len(obstacles)):
        #pdb.set_trace()
        for j in range(len(obstacles[i])):
            if j < len(obstacles[i])-1:
                lines.append(np.vstack([obstacles[i][j,:], obstacles[i][j+1,:]]))
            else:
                lines.append(np.vstack([obstacles[i][j,:], obstacles[i][0,:]]))


    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
            

                visible = True
                line = np.vstack([[vertices[i,0], vertices[i,1]], [vertices[j,0], vertices[j,1]]])

                #just debugging
                # if i == 2 and j == 3:
                #     pdb.set_trace()

                #Check for visibility
                for k in range(len(lines)):
                    if check_endpoints(line, lines[k]) == 0:
                        if check_intercept(line, lines[k]) == 1:
                            visible = False
                            break         
            
                #Remove any segments inside of boxes
                m = (line[1,1] - line[0,1])/(line[1,0] - line[0,0])
                b = line[0,1] - m*line[0,0]
                x = (line[0,0] + line[1,0]) / 2
                y = m*x+b
   
                for k in range(len(obstacles)):
                    if visible is True:

                        #construct a path and then check if inside
                        #idk why path needs to be cw
                        tp = path.Path(obstacles[k][::-1])
                        if tp.contains_point((x,y), radius=-0.001):
                            visible = False
                            break
                #plotter.plot([vertices[i,0], vertices[j,0]], [vertices[i,1], vertices[j,1]], 'r')
                if visible is True: 
                    #pdb.set_trace()
                    #print np.linalg.norm(vertices[i,:]-vertices[j,:])
                    graph[i,j] = np.linalg.norm(vertices[i,:]-vertices[j,:])
                    plotter.plot([vertices[i,0], vertices[j,0]], [vertices[i,1], vertices[j,1]], 'r')
    return graph

