function [index_min] = QueryNearestPointByPosition(x,y)
%% 寻找离当前位置距离最近的一个点
    global target_points;
    dx = target_points.x(1) - x;
    dy = target_points.y(1) - y;
    dis_min = dx^2 + dy^2;
    
    for i = 1:1:size(target_points.x) %size(target_points.x)
        d_tmp = (target_points.x(i) - x)^2 +  (target_points.y(i) - y)^2;
        if(d_tmp <= dis_min)
            dis_min = d_tmp;
            index_min = i;
%         else
%             index_min = 1;
        end
    end
end