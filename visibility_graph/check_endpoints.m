function res = check_endpoints(line1, line2)
    res = check_point(line1(1,:), line2(1,:)) +...
          check_point(line1(1,:), line2(2,:)) +...
          check_point(line1(2,:), line2(1,:)) +...
          check_point(line1(2,:), line2(2,:));
end

function res = check_point(p1, p2)
    res = (p1(1) == p2(1) && p1(2) == p2(2));
end