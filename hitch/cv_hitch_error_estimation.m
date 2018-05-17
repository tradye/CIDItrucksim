close all;
clear;

% Plot sample hitch and markers
xh = 1;
yh = 0;
theta = 5;
[x1,y1,x2,y2] = funcDualPointLocation(xh,yh, deg2rad(theta), 1, 1);
figure;
plot(xh,yh,'o',x1,y1,'o',x2,y2,'o')

theta_test = funcThetaFromCenterPointDistance(1.5, 2, 1,-1);
% Distance measurements plus gaussian error
% Question: how does variance of gaussian localization error relate to
% heading error?

% Question: how does variance of gaussian distance error relate to
% heading error? 
rng(94568);
r = normrnd(0, 0.1);
% generate a preset theta curve
% calculate the distance to the curve plus a gaussian error
[xc,yc] = funcCenterPointFromDualPoint(x1,y1,x2,y2);
dist_cp = funcDistance(-1,0, xc,yc) *(1+r);
theta_test = rad2deg(funcThetaFromCenterPointDistance(dist_cp, 2, 1, 1));

% arr_theta = [];
% arr_r = normrnd(0, 0.05, 100, 1);
% for i=1:length(arr_r)
%     r = arr_r(i);
%     [xc,yc] = funcCenterPointFromDualPoint(x1,y1,x2,y2);
%     dist_cp = funcDistance(-1,0, xc,yc);
%     arr_theta(i) = rad2deg(funcThetaFromCenterPointDistance(dist_cp, 2, 1, 1));
% end % for


arr_theta = [];
arr_r = normrnd(0, 0.05, 100, 4);
for i=1:length(arr_r)
    r = arr_r(i, :);
    [xc,yc] = funcCenterPointFromDualPoint(x1*(1+r(1)), ...
        y1*(1+r(2)),x2*(1+r(3)),y2*(1+r(4)));
    dist_cp = funcDistance(-1,0, xc,yc);
    arr_theta(i) = rad2deg(funcThetaFromCenterPointDistance(dist_cp, 2, 1, 1));
end % for

% Question: how does bias in distance error relate to
% heading error? 
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

function theta = funcThetaFromCenterPointDistance(dist_cp, ...
    dist_cam_to_hitch, radius, dir)
    a = radius;
    b = dist_cp;
    c = dist_cam_to_hitch;
    theta = acos((c^2 + a^2 - b^2)/(2*c*a));
    if dir < 0
        theta = -theta;
    end
end % funcThetaFromCenterPointDistance