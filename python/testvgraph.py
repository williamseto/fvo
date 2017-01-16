#!/usr/bin/env python

import numpy as np
import vgraph
import pdb

vertices = np.genfromtxt('vertices.txt')


obstacles = []

obstacles.append(np.genfromtxt('obs1.txt'))
obstacles.append(np.genfromtxt('obs2.txt'))
obstacles.append(np.genfromtxt('obs3.txt'))
obstacles.append(np.genfromtxt('obs4.txt'))



graph = vgraph.vgraph(obstacles, vertices, None)

np.savetxt('gr.txt', graph, delimiter=' ', fmt='%1.6f')