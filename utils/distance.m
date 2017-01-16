function d = distance(x,y)
%
% Calculate distances between two vectors x,y
% x_{n*m} , y_{n*k} => distance_{m*k}
%
% Amir Rahmani
% December 2009

d = sqrt( repmat( sum(x.*x)',1,size(y,2) ) + repmat( sum(y.*y),size(x,2),1) - 2*x'*y );