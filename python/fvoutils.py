#!/usr/bin/env python

import numpy as np
from boat import Boat
from scipy.spatial import ConvexHull
from matplotlib import path
import pdb

def get_cone(ownship, template, contact):

    expanded = np.array(Boat.expand(ownship, contact, template, 1.0))

    k = ConvexHull(expanded)


    #hull_indices = np.unique(k.simplices.flat)
    hull_indices = k.vertices
    hull_pts = expanded[hull_indices, :]

    tp = path.Path(hull_pts[::-1])
    if tp.contains_point((ownship.x, ownship.y), radius=-0.001):
        x = np.array([[-999999999, 0, 999999999]]).T
        y = np.array([[999999999, -999999999, 999999999]]).T
    else: 
        besti1 = 0
        besti2 = 0
        best_angle = 0
        xy = ownship.loc() #[1x2]
        

        for i in range(len(expanded)):
            for j in range(len(expanded)):

                v1 = expanded[i,:] - xy
                v2 = expanded[j,:] - xy


                x1 = v1[0]
                y1 = v1[1]
                x2 = v2[0]
                y2 = v2[1]
                angle = np.fmod(np.arctan2(x1*y2-x2*y1,x1*x2+y1*y2), 2*np.pi)

                if angle > best_angle:
                    best_angle = angle
                    besti1 = i
                    besti2 = j

        vel = contact.velocity()
        k = 80000

        dv1 = expanded[besti1,:] - xy    
        th = np.arctan2(dv1[1], dv1[0])
        p1 = np.array([k*np.cos(th), k*np.sin(th)]) + xy

        dv2 = expanded[besti2,:] - xy
        th = np.arctan2(dv2[1], dv2[0])
        p2 = np.array([k*np.cos(th), k*np.sin(th)]) + xy

        #return as column vectors
        x = np.array(np.matrix([ownship.x + vel[0], p1[0], p2[0]]).T)
        y = np.array(np.matrix([ownship.y + vel[1], p1[1], p2[1]]).T)

    return np.hstack([x,y])


def get_velspace(ownship, cone_list, goal, max_vel):


    # Set number of samples
    d_phi = np.pi/75
    d_vel = -5

    x_boat = ownship.x
    y_boat = ownship.y
    th_boat = ownship.theta

    x_scatter = np.array([])
    y_scatter = np.array([])

    for phi in np.arange(-np.pi/3,np.pi/3,d_phi):
        for vel in np.arange(max_vel,d_vel,d_vel):
            x = x_boat + vel * np.cos(th_boat + phi)
            y = y_boat + vel * np.sin(th_boat + phi)

            feasible = True
        
            for i in np.arange(0,cone_list.shape[1], 2):
                base = np.array([cone_list[0,i], cone_list[0,i+1]])
                a = np.append([cone_list[1,i], cone_list[1,i+1]] - base,0)
                b = np.append([cone_list[2,i], cone_list[2,i+1]] - base,0)
                c = np.append(np.array([x, y]) - base, 0)
                
                ab = np.cross(a, b)
                ac = np.cross(a, c)
                bc = np.cross(b, c)

                if ab[2] < 0:
                    if ac[2] < 0 and bc[2] > 0:
                        feasible = False
                        break
                else:
                    if ac[2] > 0 and bc[2] < 0:
                        feasible = False
                        break
            
            if feasible is True:
                x_scatter = np.append(x_scatter, x)
                y_scatter = np.append(y_scatter, y)

    vel_set = np.append([x_scatter], [y_scatter], axis=0).T

    # Get Best Velocity from velocity set and goal velocity
    best_dist = 999999999
    vel_form = [0, 0]

    for idx in range(len(vel_set)):
        vel = vel_set[idx, :]
        dist = np.linalg.norm(goal-vel)
        if dist < best_dist:
           vel_form = vel
           best_dist = dist

    return vel_form
