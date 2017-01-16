function [x, y] = get_cone(ownship, template, contact)

    expanded = Boat.expand(ownship, contact, template, 1.0)';
%     scatter(expanded(1,:), expanded(2,:))
%     pause
    
    xp = expanded(1,:);
    yp = expanded(2,:);
    k = convhull(xp, yp);

    if inpolygon(ownship.x, ownship.y, xp(k), yp(k))
        x = [-999999999 0 999999999];
        y = [999999999 -999999999 99999999];
    else   
        besti1 = 0;
        besti2 = 0;
        best_angle = 0;
        xy = ownship.loc();
        
        % TODO - Fix
        for i = 1:length(expanded)
            for j = 1:length(expanded)

                v1 = expanded(:, i)' - xy;
                v2 = expanded(:, j)' - xy;

                x1 = v1(1);
                y1 = v1(2);
                x2 = v2(1);
                y2 = v2(2);
                angle = rem(atan2(x1*y2-x2*y1,x1*x2+y1*y2), 2*pi);

                if angle > best_angle
                    best_angle = angle;
                    besti1 = i;
                    besti2 = j;
                end
            end
        end

        vel = contact.velocity();
        k = 80000;

        dv1 = expanded(:, besti1)' - xy;    
        th = atan2(dv1(2), dv1(1));
        p1 = [k*cos(th) k*sin(th)] + xy;

        dv2 = expanded(:, besti2)' - xy;
        th = atan2(dv2(2), dv2(1));
        p2 = [k*cos(th) k*sin(th)] + xy;

        x = [ownship.x + vel(1), p1(1), p2(1)];
        y = [ownship.y + vel(2), p1(2), p2(2)];
    end
end