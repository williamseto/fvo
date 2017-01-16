function [ graph ] = vgraph( obstacles, vertices )

graph = zeros(length(vertices), length(vertices));

lines = {};
idx = 1;

for i = 1:length(obstacles)
    for j = 1:length(obstacles{i})
        if j < length(obstacles{i})
            lines{idx} = [obstacles{i}(j,:); obstacles{i}(j+1,:)];
        else
            lines{idx} = [obstacles{i}(j,:); obstacles{i}(1,:)];            
        end
        idx = idx + 1;
    end
end

for i = 1:length(vertices)
    for j = 1:length(vertices)
        if i ~= j
            
            visible = true;
            line = [vertices(i,1) vertices(i,2); vertices(j,1) vertices(j,2)];

            % Check for visibility
            for k = 1:length(lines)    
                if check_endpoints(line, lines{k}) ==0
                    if check_intercept(line, lines{k}) == 1
                        visible = false;
                        break
                    end
                end
            end           
            
            % Remove any segments inside of boxes
            m = (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
            b = line(1,2) - m*line(1,1);
            x = (line(1,1) + line(2,1)) / 2;
            y = m*x+b;
                       
            for k = 1:length(obstacles)
                if visible
                    [in, on] = inpolygon(x,y,obstacles{k}(:,1),obstacles{k}(:,2));
                    if in && ~on
                        visible = false;
                        break
                    end
                end
            end
            
            if visible       
                graph(i,j) = norm(vertices(i,:)-vertices(j,:));
                plot([vertices(i,1) vertices(j,1)], [vertices(i,2) vertices(j,2)], 'r');
            end
        end
    end
end

end

