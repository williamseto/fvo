function [c] = centroid(pSwarm, nBoats)
%CENTROID Summary of this function goes here
%   Detailed explanation goes here

totalX = 0;
totalY = 0;

for idx = 1:1:nBoats
   totalX = totalX + pSwarm(idx, 1);
   totalY = totalY + pSwarm(idx, 2);
end

c = [totalX/nBoats totalY/nBoats];

end
