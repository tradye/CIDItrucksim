clear;

thetas = linspace(0, deg2rad(45), 45);
dualpoints = [];
for i=1:length(thetas)
    theta = thetas(i);
    [dualpoints(i,1), ...
        dualpoints(i,2), ...
        dualpoints(i,3), ...
        dualpoints(i,4)] = funcDualPointLocation(0.835, 0, ...
        -theta, 0.6, 0.835); 
end
dualpoints_distances = [];
for i=1:length(thetas)
    dualpoints_distances(i,1) = ...
        funcDistance(-0.5-1.919+0.835, 0, dualpoints(i,1), dualpoints(i,2));
    dualpoints_distances(i,2) = ...
        funcDistance(-0.5-1.919+0.835, 0, dualpoints(i,3), dualpoints(i,4));       
end
figure;



function [x1,y1,x2,y2] = funcDualPointLocation(hitch_x, hitch_y, ...
    theta, point_distance, radius)
    center_point_x = hitch_x - radius*cos(theta);
    center_point_y = hitch_y + radius*sin(theta);
    x1 = center_point_x + point_distance/2 * sin(theta);
    x2 = center_point_x - point_distance/2 * sin(theta);
    y1 = center_point_y + point_distance/2 * cos(theta);
    y2 = center_point_y - point_distance/2 * cos(theta);
end % funcDualPointLocation

function [xc, yc] = funcCenterPointFromDualPoint(x1,y1,x2,y2)
    xc = (x1 + x2)/2;
    yc = (y1 + y2)/2;
end % funcCenterPointFromDualPoint

function dist = funcDistance(x0,y0,x1,y1)
    dist = sqrt((x1-x0)^2 + (y1-y0)^2);
end % funcDistance

function orientation = funcOrientationFromDualPoint(x1,y1,x2,y2)
    % returns 1 if x1 is "behind" x2
    epsilon = 0.0001;
    if x1 > x2 + epsilon
        orientation = 1;
    elseif x1 + epsilon < x2
        orientation = -1;
    else
        orientation = 0;
    end
end %funcOrientationFromDualPoint

function theta = funcThetaFromCenterPointDistance(dist_cp, ...
    dist_cam_to_hitch, radius, dir)
    a = radius;
    b = dist_cp;
    c = dist_cam_to_hitch;
    if dist_cp + radius < dist_cam_to_hitch
        theta = 0;
    else
        theta = acos((c^2 + a^2 - b^2)/(2*c*a));
    end
    if dir < 0
        theta = -theta;
    end
end % funcThetaFromCenterPointDistance