#!/usr/bin/env python

import numpy as np

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pdb


class Boat:

   x = 0
   y = 0
   theta = 0
   vel = 0
   scale = 0
   points = []
   default_points = []
   formation_pts = []
   otype = 0

   boat_pts = np.array([[0.5581, 0.5116, 0.3721, 0.1628, -0.3953, -0.4419, -0.3953, 0.1628, 0.3721, 0.5116], \
                        [0, 0.0698, 0.1395, 0.1860, 0.1628, 0, -0.1628, -0.1860, -0.1395, -0.0698]])

   contact_pts = np.array([[3.0, 3.0, -3.0,  -3.0], \
                           [-1.5, 1.5,  1.5,  -1.5]])

   fat_pts = np.array([[-1.5,  1.5,    1.5,   -1.5], \
                       [5.0,  5.0,   -5.0,   -5.0]])
       
   skinny_pts = np.array([[5.0,  5.0, -5.0,   -5.0], \
                          [-1.5,  1.5,  1.5,   -1.5]])
        
   square_pts = np.array([[4, 4, -4, -4], \
                          [-4, 4,  4, -4]])
        
   min_pts = np.array([[1.5, 1.5, -1.5, -1.5], \
                       [-1.5, 1.5,  1.5, -1.5]])

   diamond_pts = np.array([[3, 0,  -3,  0], \
                           [0, 3,   0, -3]])

   same_pts = np.array([[0, 0, 0, 0], \
                        [0, 0, 0, 0]])

   switcher = {
        'boat': boat_pts,
        'formation': same_pts,
        'contact': contact_pts,
        'static' : contact_pts,
   }

   def __init__(self, x, y, theta, vel, scale, itype, icolor, transparency):
      self.x = x
      self.y = y
      self.theta = theta
      self.vel = vel
      self.otype = itype
      self.default_points = self.switcher.get(itype, self.contact_pts)
      self.scale = scale
      self.default_points = self.default_points * scale
      self.points = self.default_points
      self.h_boat = Polygon(self.default_points.T, True, color=icolor, alpha=transparency)


   def plot(self):
      R = np.array([[np.cos(self.theta), -np.sin(self.theta)], \
                    [np.sin(self.theta),  np.cos(self.theta)]])

      xy = np.array([[self.x], [self.y]])

      self.points = np.dot(R, self.default_points) + np.tile(xy, (1, self.points.shape[1]))
      self.h_boat.xy = self.points.T

   def update(self, dt, vel=None):
      if vel is not None:
         if np.linalg.norm(vel) != 0:
            h = np.matrix([[np.cos(self.theta)], [np.sin(self.theta)]])

            dw = .2 * np.matrix(vel/np.linalg.norm(vel)) * np.append([[0, -1]], [[1, 0]], axis=0) * h

            self.vel = np.linalg.norm(vel)
            self.theta = self.theta + float(dw)
         else:
            self.vel = 0

                        
      self.x = self.x + self.vel * np.cos(self.theta)*dt
      self.y = self.y + self.vel * np.sin(self.theta)*dt
      self.theta = self.theta

   def update_formation(self, otype):

      if otype is 1:
         self.default_points = self.fat_pts * self.scale;
      elif otype is 2:
         self.default_points = self.skinny_pts * self.scale;
      elif otype is 3:
         self.default_points = self.diamond_pts * self.scale;
      else:
         print('ERROR UPDATING FORMATION')

               
   def loc(self):
      return np.array([self.x, self.y])

        
   #Creates a rotational matrix for body frame
   def rotation(self):
      return np.array([[np.cos(self.theta), -np.sin(self.theta)], \
                       [np.sin(self.theta),  np.cos(self.theta)]])
        
   #Returns a velocity vector
   def velocity(self):
      return np.array([self.vel * np.cos(self.theta), self.vel * np.sin(self.theta)])
        
   def pose(self):
      return np.array([self.x, self.y, self.theta])
    
   @staticmethod 
   def expand(b1, b2, otype, scale):
      expanded_pts = np.array([])
      
      if otype == 1:
         pts = b1.fat_pts * b1.scale
      elif otype == 2:
         pts = b1.skinny_pts * b1.scale
      elif otype == 3:
         pts = b1.diamond_pts * b1.scale
      elif otype == 4:
         pts = b1.square_pts * b1.scale
      elif otype == 5:
         pts = b1.boat_pts * 20
      elif otype == 6:
         pts = b1.min_pts * b1.scale
      else:
         print('ERROR EXPANDING', otype)         
         pts = b1.fat_pts
      
      for i in range(b1.points.shape[1]):

         P = np.dot(b1.rotation(),pts) * scale + np.tile(np.matrix(b2.points)[:,i], (1, b2.points.shape[1]))

         expanded_pts = np.vstack([expanded_pts, P.T]) if expanded_pts.size else P.T



      return expanded_pts
