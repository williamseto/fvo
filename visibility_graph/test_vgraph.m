clf
clear
clc

figure(1)
axis([-10 40 -10 40])

t1 = [-1.5  1.5    1.5   -1.5
       5.0  5.0   -5.0   -5.0];

t2 = [5.0  5.0 -5.0   -5.0   
     -1.5  1.5  1.5   -1.5]*.5;

obs1 =  [5  5; 
         5  10; 
         10 10; 
         10 5];
     
obs2 =  [0 0; 
         0 1; 
         1 1; 
         1 0];

obs1 = bsxfun(@plus, obs1, [3 0]);
obs2 = bsxfun(@plus, obs1, [2 10]);
obs3 = bsxfun(@plus, obs2, [12 3]);
obs4 = bsxfun(@plus, obs2, [7 12]);

obstacles = {obs1, obs2, obs3, obs4};
vertices = [];

hold on;
for i = 1:length(obstacles)
    patch(obstacles{i}(:,1), obstacles{i}(:,2), 'b', 'FaceAlpha', .3);
    vertices = [vertices; obstacles{i}];
end
vertices = [vertices];

graph = vgraph(obstacles, vertices);

% Plot Shortest Path
sparse_graph = sparse(graph);
[dist,path,pred] = graphshortestpath(sparse_graph,1,18);

for i = 1:length(path)-1
%     plot([vertices(path(i),1) vertices(path(i+1),1)], [vertices(path(i),2) vertices(path(i+1),2)], 'g')
end


