function [points] = move2dBoats(handle, pose, scale)
% Plots n number of boats in 2D space
%
%move2dBoats(handle, pose, scale)
%handle - handle for the plot (I think?)
%pose   - positions of all boats in formation
%scale  - size to scale boat by.

% Author Dr. Amir Rahmani
% Modified by Matt Epperson

nBoats = size(pose);
nBoats = nBoats(1,1);

boat = getBoatShape;

if ~exist('scale','var')
    scale{1} = 1;
end

if length(scale) == 1
    for i = 2:nBoats
        scale{i} = scale{1};
    end
end

for i = 1:nBoats
    x =  pose(i,1);
    y =  pose(i,2);
    th = pose(i,3);

    R = [cos(th) -sin(th); sin(th) cos(th)];    
    P = scale{i} * R * boat + [x; y]*ones(1,size(boat,2));
    set(handle{i}, 'XData', P(1,:), 'YData', P(2,:));
end
    points = P';
end