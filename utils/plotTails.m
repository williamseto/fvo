function tH = plotTails(tH,xHistory,tailLength,color)

nV = length(xHistory);

if ~exist('color','var')
        color{1} = 'b';
end

if length(color) == 1
    for i = 2:nV
        color{i} = color{1};
    end
end


flag = (length(tH) == 0);
l = size(xHistory{1},1);

for i = 1:nV

    tailX = xHistory{i}(max(1,l-tailLength+1):l,1);
    tailY = xHistory{i}(max(1,l-tailLength+1):l,2);
    
    if flag
        tH{i} = plot(tailX, tailY, '.');
        set(tH{i},'Color',color{i});
        %hold on
    else
        set(tH{i},'XData',tailX,'YData',tailY);
    end
end

end
        