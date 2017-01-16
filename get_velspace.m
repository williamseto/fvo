function [ vel_form ] = get_velspace(ownship, cone_list, goal, max_vel)
%GET_VELSPACE Summary of this function goes here
%   Detailed explanation goes here

% Set number of samples
d_phi = pi/75;
d_vel = -5;

x_boat = ownship.x;
y_boat = ownship.y;
th_boat = ownship.theta;

x_scatter = [];
y_scatter = [];

for phi = -pi/3:d_phi:pi/3
    for vel = max_vel:d_vel:0
        x = x_boat + vel * cos(th_boat + phi);
        y = y_boat + vel * sin(th_boat + phi);

        feasible = true;
        
        for i = 1:2:size(cone_list,2)
            base = [cone_list(1,i), cone_list(1,i+1)];
            a = [[cone_list(2,i), cone_list(2,i+1)] - base,0];
            b = [[cone_list(3,i), cone_list(3,i+1)] - base,0];
            c = [[x, y] - base, 0];
            
            ab = cross(a, b);
            ac = cross(a, c);
            bc = cross(b, c);

            if ab(3) < 0
                if ac(3) < 0 && bc(3) > 0
                    feasible = false;
                    break 
                end
            else
                if ac(3) > 0 && bc(3) < 0
                    feasible = false;
                    break
                end
            end           
        end
        
        if feasible
            x_scatter = [x_scatter x];
            y_scatter = [y_scatter y];
        end
    end
end

vel_set = [x_scatter', y_scatter'];
% scatter(x_scatter, y_scatter);
% pause

% Get Best Velocity from velocity set and goal velocity
best_dist = 999999999;
vel_form = [0 0];

for idx = 1:1:size(vel_set)
    vel = vel_set(idx, :);
    dist = norm(goal-vel);
    if dist < best_dist
       vel_form = vel;
       best_dist = dist;
    end
end

end

