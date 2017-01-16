#!/usr/bin/env python

import numpy as np
import boat

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pdb
from scipy.spatial import ConvexHull
import vgraph
from scipy.sparse.csgraph import csgraph_from_dense, dijkstra
import fvoutils
from munkres import Munkres
from scipy.spatial.distance import cdist

plt.ion()
fig, ax = plt.subplots()
plt.show()
ax.axis('auto')

#Setup Formation and Obstacles

dat = np.genfromtxt('../testcases/random.dat', delimiter=',')

ownship = boat.Boat(dat[0,0], dat[0,1], dat[0,2], dat[0,3], dat[0,4], 'formation', 'b', .3)
ax.add_patch(ownship.h_boat)

contact_list = []
for i in range(1,dat.shape[0]):
   contact_list.append(boat.Boat(dat[i,0], dat[i,1], dat[i,2], dat[i,3], dat[i,4], 'contact', 'r', .3)) 
   contact_list[i-1].plot()
   ax.add_patch(contact_list[i-1].h_boat)
   #h_cones{i-1} = patch(0, 0, 'k', 'FaceAlpha', .3);

fig.canvas.draw()
ax.axis('auto')

formation_pref = [9, 8, 7, 6];
nFormations = 4;

#Setup Team Ships
form_pts = ownship.points;
team_list = [];

for i in range(form_pts.shape[1]):
    team_list.append(boat.Boat(form_pts[0,i], form_pts[1,i]-1, np.pi/2, 5, 25, 'boat', 'g', 1))
    team_list[i].plot()
    ax.add_patch(team_list[i].h_boat)
    #h_arrows{i} = drawArrow([0 0], [0 0], 'r');

fig.canvas.draw()
ax.axis('auto')

#h_formation = patch(0, 0, 'g', 'FaceAlpha', .3);
#h_waypoints = plot(0,0,'ro');

# Path planning using visibility graph
# goal = [70 250];
goal = [10, 500]
obstacles = []
vertices = [0, 0]
path = []

for i in range(len(contact_list)):
    form_pts = boat.Boat.expand(ownship, contact_list[i], 6, 1.0);
    k = ConvexHull(form_pts)


    #hull_indices = np.unique(k.simplices.flat)
    hull_indices = k.vertices
    hull_pts = form_pts[hull_indices, :]

    # plt.plot(form_pts[:, 0], form_pts[:, 1], 'ko', markersize=10)
    # plt.plot(hull_pts[:, 0], hull_pts[:, 1], 'ro', alpha=.25, markersize=20)
    # plt.show()
    # pdb.set_trace()


    obstacles.append(np.array(hull_pts))
    vertices = np.vstack([vertices, obstacles[i]])


vertices = np.vstack([vertices, goal])

graph = vgraph.vgraph(obstacles, vertices, plt)

sparse_graph = csgraph_from_dense(graph)

path = []
(dist, preds) = dijkstra(sparse_graph, return_predecessors=True)

curr = len(vertices)-1
while curr != 0:
    path.append(curr)
    curr = preds[0, curr]
path.append(0)
path = path[::-1]


#Plot Shortest Path
for i in range(len(path)-1):
    plt.plot([vertices[path[i],0], vertices[path[i+1],0]], [vertices[path[i],1], vertices[path[i+1],1]], 'g')


pindex = 0
path = vertices[path,:]


#RUN
run_time = 125
dt = .2

for t in np.arange(0.2,run_time,dt):
    
    #Formation Selection
        
    if np.linalg.norm(path[pindex,:] - ownship.loc()) < 20:
       pindex = pindex + 1

    
    v_pref = (path[pindex,:] - ownship.loc()) / np.linalg.norm(path[pindex,:] - ownship.loc())

    best_formation = 0
    best_cost = 0
    best_vel = ownship.loc()

    for i in range(1, nFormations+1):
        #Update Contacts and Cones
        cone_list = np.array([])

        for n in range(len(contact_list)):
            xy = fvoutils.get_cone(ownship, i, contact_list[n])
            #cone_list = [cone_list [x' y']];
            cone_list = np.hstack([cone_list, xy]) if cone_list.size else xy
            #set(h_cones{n}, 'XData', x, 'YData', y);

        # Get Velocity Set
        vel_form = fvoutils.get_velspace(ownship, cone_list, path[pindex,:], 20)

        cost = formation_pref[i-1] * np.dot(v_pref, (vel_form - ownship.loc()) / np.linalg.norm(vel_form - ownship.loc()))
        
        if cost > best_cost:
            best_formation = i
            best_cost = cost
            best_vel = vel_form

        #drawArrow(ownship.loc(), best_vel, 'r', h_arrow);

    # Update 'formation' aka ownship
    ownship.update_formation(best_formation)
    ownship.update(dt, best_vel - ownship.loc())
    ownship.plot()

    # Trajectory tracking for each boat
    form_pts = ownship.points
    
    # Compute optimal assignments in formation using hungarian alg
    boat_pts = np.array([team_list[0].loc(), team_list[1].loc(), team_list[2].loc(), team_list[3].loc()])

    A = cdist(form_pts.T, boat_pts, 'euclidean')
    m = Munkres()
    C = m.compute(A)
    C = [el[1] for el in C]
    form_pts = form_pts[:,C]
    
    # Extrapolate formation positions into future
    p_form = np.array([[ownship.x, ownship.y]])
    f_vel  = np.array([[np.cos(ownship.theta) * ownship.vel, np.sin(ownship.theta) * ownship.vel]])

    t_extrapolated = .5*(np.max(cdist(p_form, boat_pts)) + np.max(cdist(p_form, form_pts.T))) / max(.1,ownship.vel)
    
    form_pts_p_delta = f_vel * t_extrapolated
    form_pts_p = form_pts + form_pts_p_delta.T

                
#     % Update plots showing formation points
#     set(h_waypoints, 'XData', form_pts_p(1,:), 'YData', form_pts_p(2,:));

    hull = [];
    for i in range(len(team_list)):
        # Difference between current position and extrapolated position
        # minus the extrapulated distance between center of formation
        ddiff = np.linalg.norm(form_pts_p[:,i] - team_list[i].loc()) - np.linalg.norm(form_pts_p_delta)
        
        # Calculate new velocity
        angle = (form_pts_p[:,i] - team_list[i].loc()) / np.linalg.norm(form_pts_p[:,i] - team_list[i].loc())
        vel_des = (ownship.vel + ddiff/t_extrapolated) * angle
        
# %         cone_list = [];
# %         for n=1:length(contact_list)
# %             [x, y] = get_cone(team_list{i}, 4, contact_list{n}); 
# %             cone_list = [cone_list [x' y']];
# %         end
# %         
# %         vel = get_velspace(team_list{i}, cone_list, vel_des + team_list{i}.loc(), 120);
# %         drawArrow(team_list{i}.loc(), vel, 'r', h_arrows{i}); 
# %         team_list{i} = team_list{i}.update(dt, vel - team_list{i}.loc());
        
        
        # Update team
        team_list[i].update(dt, vel_des)
        team_list[i].plot()
        
#         hull = [hull; [team_list{i}.x team_list{i}.y]];
#         set(h_formation, 'XData', hull(:,1), 'YData', hull(:,2));
    
    
#     % Update Contacts
#     for n=1:length(contact_list);
#         contact_list{n} = contact_list{n}.plot();
#         contact_list{n} = contact_list{n}.update(dt);
#     end
    fig.canvas.draw()

raw_input("press key to quit")


