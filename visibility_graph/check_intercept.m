function [ intercepts ] = check_intercept( line1, line2 )

m1 = slope(line1);
m2 = slope(line2);

if m1 == m2
   intercepts = 0;
   return;
end

if m1 == Inf || m1 == -Inf
    line1(1,1) = line1(1,1) + .001;
    m1 = slope(line1);
end

if m2 == -Inf || m2 == Inf
    line2(1,1) = line2(1,1) + .001;
    m2 = slope(line2);
end

b1 = intercept(line1,m1);
b2 = intercept(line2,m2);

xintersect = (b2-b1)/(m1-m2);
yintersect = m1*xintersect + b1;

intercepts = isPointInside(xintersect,line1) && isPointInside(xintersect,line2);
end

%% Helper functions
function inside = isPointInside(xint,myline)
    inside = (xint >= myline(1,1) && xint <= myline(2,1)) || ...
             (xint >= myline(2,1) && xint <= myline(1,1));
end

function m = slope(line)
    m = (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
end

function b = intercept(line, m)
    b = line(1,2) - m*line(1,1);
end