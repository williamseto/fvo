%% Setup Window
clear, clc, clf

figure(1);
window = [500, 500]; %display window; use [] to show all.

generateMovie = false;
if generateMovie
    videoObj = VideoWriter('media/random_team.mp4', 'MPEG-4');
    videoObj.FrameRate = 15; %default: 30 fps
    videoObj.Quality = 100; %default: 75
    open(videoObj);
end


%% Setup Formation and Obstacles
hold on

dat = csvread('testcases/random.dat');

ownship = Boat(dat(1,1),dat(1,2),dat(1,3),dat(1,4),dat(1,5), 'formation', 'b', .3);
h_arrow = drawArrow([0 0], [0 0], 'r');

for i = 2:size(dat)
   contact_list{i-1} = Boat(dat(i,1),dat(i,2),dat(i,3),dat(i,4),dat(i,5),'contact', 'r', .3); 
   contact_list{i-1} = contact_list{i-1}.plot();
   h_cones{i-1} = patch(0, 0, 'k', 'FaceAlpha', .3);
end

nFormations = 4;
% formation_pref{1} = 20;
formation_pref{1} = 9;
formation_pref{2} = 8;
formation_pref{3} = 7;
formation_pref{3} = 6;


%% Setup Team Ships
form_pts = ownship.points;
team_list = {};

for i = 1:size(form_pts,2)
    team_list{i} = Boat(form_pts(1,i), form_pts(2,i)-1, pi/2, 5, 25, 'boat', 'g', 1);
    team_list{i} = team_list{i}.plot();
    h_arrows{i} = drawArrow([0 0], [0 0], 'r');
end

h_formation = patch(0, 0, 'g', 'FaceAlpha', .3);
h_waypoints = plot(0,0,'ro');

%% TIPP
% goal = [70 250];
goal = [10 500];
obstacles = {};
vertices = [0 0];
path = [];

for i = 1:length(contact_list)
    form_pts = Boat.expand(ownship, contact_list{i}, 6, 1.0);
    k = convhull(form_pts, 'simplify', true);
    k = k(1:length(k)-1);
%     scatter(form_pts(k,1),form_pts(k,2));
    obstacles{i} = form_pts(k,:);
    vertices = [vertices; obstacles{i}];
end

vertices = [vertices; goal];
graph = vgraph(obstacles, vertices);

% Plot Shortest Path
sparse_graph = sparse(graph);
[dist,path,pred] = graphshortestpath(sparse_graph,1,length(vertices));

for i = 1:length(path)-1
    plot([vertices(path(i),1) vertices(path(i+1),1)], [vertices(path(i),2) vertices(path(i+1),2)], 'g')
end

pindex = 1;
path = vertices(path,:);

%% RUN
run_time = 125;
dt = .2;
pause

for t = dt:dt:run_time*dt
    
    pause(.1)
    % Formation Selection
        
    if norm(path(pindex,:) - ownship.loc()) < 20
       pindex = pindex + 1;
    end
    
    v_pref = (path(pindex,:) - ownship.loc()) / norm(path(pindex,:) - ownship.loc())
    
    best_formation = 0;
    best_cost = 0;
    best_vel = ownship.loc();
    
    for i = 1:1:nFormations
        % Update Contacts and Cones
        cone_list = [];

        for n=1:length(contact_list)
            [x, y] = get_cone(ownship, i, contact_list{n}); 
            cone_list = [cone_list [x' y']];
%             set(h_cones{n}, 'XData', x, 'YData', y);
        end
        
        % Get Velocity Set
        vel_form = get_velspace(ownship, cone_list, path(pindex,:), 20);
        cost = formation_pref{i} * dot(v_pref, (vel_form - ownship.loc()) / norm(vel_form - ownship.loc()));
        
        if cost > best_cost
            best_formation = i;
            best_cost = cost;
            best_vel = vel_form;
        end
        
    end
    
    drawArrow(ownship.loc(), best_vel, 'r', h_arrow);

    % Update 'formation' aka ownship
    ownship = ownship.update_formation(best_formation);
    ownship = ownship.update(dt, best_vel - ownship.loc());
    ownship = ownship.plot();

    % Trajectory tracking for each boat
    % Copied from - Simulating and Evaluating the Local Behavior of Small Pedestrian Groups
    form_pts  = ownship.points;
    
    % Compute optimal assignments in formation using hungarian - Thanks Amir!
    boat_pts = [team_list{1}.loc; team_list{2}.loc; team_list{3}.loc; team_list{4}.loc];
    A = distance(form_pts, boat_pts');
    [C, T] = hungarian(A);
    form_pts = form_pts(:,C);
    
    % Extrapolate formation positions into future
    p_form = [ownship.x ownship.y];
    f_vel  = [cos(ownship.theta) * ownship.vel, sin(ownship.theta) * ownship.vel];

    t_extrapulated = .5*(max(distance(p_form', boat_pts')) + max(distance(p_form', form_pts))) / max(.1,ownship.vel);
    
    form_pts_p_delta = f_vel * t_extrapulated;
    form_pts_p = bsxfun(@plus,form_pts, form_pts_p_delta');
                
    % Update plots showing formation points
    set(h_waypoints, 'XData', form_pts_p(1,:), 'YData', form_pts_p(2,:));

    hull = [];
    for i = 1:length(team_list)
        % Difference between current position and extrapulated position
        % minus the extrapulated distance between center of formation
        ddiff = norm(form_pts_p(:,i)' - team_list{i}.loc()) - norm(form_pts_p_delta);
        
        % Calculate new velocity
        angle = (form_pts_p(:,i)' - team_list{i}.loc()) / norm(form_pts_p(:,i)' - team_list{i}.loc());
        vel_des = (ownship.vel + ddiff/t_extrapulated) * angle;
        
%         cone_list = [];
%         for n=1:length(contact_list)
%             [x, y] = get_cone(team_list{i}, 4, contact_list{n}); 
%             cone_list = [cone_list [x' y']];
%         end
%         
%         vel = get_velspace(team_list{i}, cone_list, vel_des + team_list{i}.loc(), 120);
%         drawArrow(team_list{i}.loc(), vel, 'r', h_arrows{i}); 
%         team_list{i} = team_list{i}.update(dt, vel - team_list{i}.loc());
        
        
        % Update team
        team_list{i} = team_list{i}.update(dt, vel_des);
        team_list{i} = team_list{i}.plot();
        
        hull = [hull; [team_list{i}.x team_list{i}.y]];
        set(h_formation, 'XData', hull(:,1), 'YData', hull(:,2));
    end
    
    
    % Update Contacts
    for n=1:length(contact_list);
        contact_list{n} = contact_list{n}.plot();
        contact_list{n} = contact_list{n}.update(dt);
    end
    
    drawnow
    axis([ownship.x - 300 ownship.x + 300 ownship.y - 300 ownship.y + 300]);
%     pause

    if generateMovie %write movie frame
        axisHandle = gca;
        frame = getframe(axisHandle);
        writeVideo(videoObj, frame);
    end
end


if generateMovie
  close(videoObj);
end


        
