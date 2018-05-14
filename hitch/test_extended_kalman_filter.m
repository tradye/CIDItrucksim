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

run('LoadTestPaths_draft');
target_path = pathLaneChange_double_f;
theta_profile = target_path.angle;
dt_profile = target_path.time_elapsed;
speed_profile = target_path.speedProfile;

% 1. find ideal truck and trailer path, and hitch angles
[trailer_angles, trailer_times] = funcGetTrailerAngles(theta_profile, ...
    speed_profile, 3, dt_profile);  

% 2. calculate implied measurements, given hitch angles
hitch_angles = trailer_angles - theta_profile;
% calculate matrix of dualpoint locations (Nx4 matrix)
% calculate list of centerpoint measurements and left/right (Nx1 matrix)
dualpoints = [];
centerpoints = [];
cpdistances = [];
implied_angles = [];
for i=1:length(hitch_angles)
    hitch_angle = hitch_angles(i);
    [dualpoints(i,1), ...
        dualpoints(i,2), ...
        dualpoints(i,3), ...
        dualpoints(i,4)] = funcDualPointLocation(1, 0, -hitch_angle, 1, 1);
    [centerpoints(i,1), ...
        centerpoints(i,2)] = funcCenterPointFromDualPoint(dualpoints(i,1), ...
        dualpoints(i, 2), ...
        dualpoints(i, 3), ...
        dualpoints(i, 4));
    cpdistances(i, 1) = funcDistance(-1, 0, centerpoints(i, 1), centerpoints(i, 2));    
    cpdistances(i, 2) = funcOrientationFromDualPoint(dualpoints(i,1), ...
        dualpoints(i, 2), ...
        dualpoints(i, 3), ...
        dualpoints(i, 4));
    implied_angles(i,1) = -funcThetaFromCenterPointDistance(cpdistances(i, 1), ...
        2, 1, cpdistances(i,2));
end % for i
% 3. go through measurements again and add error
r = normrnd(0, 0.05,length(dualpoints),4);
% 4. calculate "wrong" hitch angles given measurement errors
bad_dualpoints = dualpoints.*(1+r);
bad_centerpoints = [];
bad_cpdistances = [];
bad_implied_angles = [];
for i=1:length(hitch_angles)
    hitch_angle = hitch_angles(i);
    [bad_centerpoints(i,1), ...
        bad_centerpoints(i,2)] = funcCenterPointFromDualPoint(bad_dualpoints(i,1), ...
        bad_dualpoints(i, 2), ...
        bad_dualpoints(i, 3), ...
        bad_dualpoints(i, 4));
    bad_cpdistances(i, 1) = funcDistance(-1, 0, ...
        bad_centerpoints(i, 1), bad_centerpoints(i, 2));    
    bad_cpdistances(i, 2) = funcOrientationFromDualPoint(bad_dualpoints(i,1), ...
        bad_dualpoints(i, 2), ...
        bad_dualpoints(i, 3), ...
        bad_dualpoints(i, 4));
    bad_implied_angles(i,1) = -funcThetaFromCenterPointDistance(bad_cpdistances(i, 1), ...
        2, 1, bad_cpdistances(i,2));
end % for i
figure;
plot(implied_angles, bad_implied_angles);
% 5. compare and calculate error paths

% 6. use kalman filters to calculate "correct" hitch angles" 
% given measurement errors, using EKF
mu0 = 0;
Sigma0 = [1000]; 
mu1 = 0;
Sigma1 = [1000]; 
v_mu = [0];
v_Sig = [1000];
remembered_z = 0;
for i=2:length(hitch_angles)
    dt = trailer_times(i);
    mu0 = mu1;       
    Sigma0 = Sigma1;
    if mod(i,100) == 0
        z1 = [bad_implied_angles(i)];        
        remembered_z  = z1;
    else
        z1 = remembered_z;
    end
    G1 = -speed_profile(i)/3*cos(-mu0)*dt;                
    R1 = 0;
    Q1 = 0.05;
    H1 = 1;
    g = wrapToPi(v_mu(i-1)+theta_profile(i-1) + ...
        speed_profile(i)/3*sin(-mu0)*dt - ...
        theta_profile(i));
    if g > 1
       disp("Got here")
    end
    [mu1, Sigma1] = ...
         funcExtendedKalmanFilter(Sigma0, z1, G1, R1, Q1, H1, g);
    v_mu = [v_mu; mu1];
    v_Sig = [v_Sig; Sigma1];
end

v_diff = bad_implied_angles - v_mu;
close all;

x = linspace(1,length(hitch_angles), length(hitch_angles));
figure;
hold on;
subplot(2,1,1);
plot(x,bad_implied_angles, 'r', x, v_mu, 'b');
subplot(2,1,2);
plot(x,implied_angles);
hold off;

% Question: how does bias in distance error relate to
% heading error? 
function [trailer_angles, trailer_times] = funcGetTrailerAngles(angles, ...
    speeds, L, times)
    trailer_angles = [0];
    trailer_times = [0];
    trailer_speeds = [speeds(1)];
    for i=2:length(times)
        dt = times(i);
        trailer_angles(i) = wrapToPi(trailer_angles(i-1) + ...
            speeds(i)/L * sin(angles(i-1) - trailer_angles(i-1)) * dt);
        trailer_times(i) = dt;
        trailer_speeds(i) = speeds(i);
    end % for
    trailer_angles = trailer_angles';
    trailer_times = trailer_times';    
end %funcGetTrailerAngles


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