function pH = plot2dBoats(x_y_theta,scale,faceColor,edgeColor)

X = x_y_theta;
nV = size(X, 1);

p = getBoatShape;

if ~exist('faceColor','var')
        faceColor{1} = 'c';
end

if ~exist('edgeColor','var')
        edgeColor{1} = 'k';
end


if ~exist('scale','var')
    scale{1} = 1;
end

if length(faceColor) == 1
    for i = 2:nV
        faceColor{i} = faceColor{1};
    end
end

if length(edgeColor) == 1
    for i = 2:nV
        edgeColor{i} = edgeColor{1};
    end
end

if length(scale) == 1
    for i = 2:nV
        scale{i} = scale{1};
    end
end



for i = 1:nV
    x =  X(i,1);
    y =  X(i,2);
    th = X(i,3);
    
    R = [cos(th) -sin(th); sin(th) cos(th)];
    P = scale{i} * R * p + [x; y]*ones(1,size(p,2));
    
    pH{i} = patch(P(1,:), P(2,:), faceColor{i});
    set(pH{i},'EdgeColor',edgeColor{i});
    %hold on
end

hold off
end