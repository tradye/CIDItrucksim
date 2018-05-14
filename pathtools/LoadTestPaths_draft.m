%  ██████╗██╗██████╗ ██╗
% ██╔════╝██║██╔══██╗██║
% ██║     ██║██║  ██║██║
% ██║     ██║██║  ██║██║
% ╚██████╗██║██████╔╝██║ LoadTestPaths.m
%  ╚═════╝╚═╝╚═════╝ ╚═╝
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
% The purpose of this file is to load a series of test paths
% for another MATLAB program.
%
% Usage: run('LoadTestPaths')
%
% cidiPath Syntax:
%   cidiPath(poses, speedStart, speedEnd, speedMax, deltaPoints)
%
% - Poses is a (N x 3) matrix. Each row is:
%               (global_x_position, global_y_position, heading_angle)
%   The first pose should be (0, 0, 0).
% - speedStart, speedEnd, speedMax are speed markers for the path in m/s.
%   Use the same number for all three, for now.
% - deltaPoints is the number of estimated points on the Spline between
%   each point. Not that important for now. Use 250
% 
% (Note: if running script independently, add "clear" command to clean
%    workspace)
% (Note: Program requires at least 4 rows in Pose matrices)
%
% TODO: Xu Hui -> Create cidiPath instances that reflect several
%   sample drivable paths. Paths should be 5-20 seconds long based on
%   the indicated speeds.
%   You can use for loops

% close all;
% clear;

% % TODO: Straight Path

% % TODO: Straight Diagonal Path

% % Example: Single Lane Change: Slow
pathPoses = [0 0 0; 10 0 0; 20 0 0; 30 10 45; 40 20 45];
pathPoses = [pathPoses; 50 30 45; 60 30 0; 70 30 0; 80 30 0];
pathPoses = [pathPoses; 90 30 0; 100 30 0];
pathPoseLaneChange = [0 0 0;30 0 0;60 0 0;100 12 0; ...
    120 17.32 30;150 30 30;180 47.32 30; ...
    210 56 0;240 60 0;270 60 0; ...
    300 60 0];
pathLaneChange = cidiPath(pathPoseLaneChange, 30, 30, 30, 1000);
disp("Total Distance (m):")
disp(sum(pathLaneChange.distance_elapsed));
disp("Total Time (t):")
disp(sum(pathLaneChange.time_elapsed));
figure
pathLaneChange.plot(); 

pathPoseLaneChange_double_f=[0 0 0; 30 0 0; ...
    60 0 0; 90 17.32 30; ...
    120 30 30; 150 47.32 30; ...
    180 53 0; 210 42.68 -30; ...
    240 30 -30; 270 12.68 -30; ...
    300 0 -30; 330 -17.32 -30; ...
    360 -30 -30; 390 -47.32 -30; ...
    420 -60 0; 450 -60 0; ...
    480 -60 0; 510 -47.32 30; ...
    540 -30 30; 570 -17.32 30; ...
    600 0 0; 630 0 0; ...
    660 0 0];
scaleMatrix = ones(size(pathPoseLaneChange_double_f));
scaleMatrix(:,2) = 0.2;
pathPoseLaneChange_double_f = pathPoseLaneChange_double_f.*scaleMatrix;
pathLaneChange_double_f = cidiPath(pathPoseLaneChange_double_f,30, 30, 30, 1000);
disp("Total Distance (m):")
disp(sum(pathLaneChange_double_f.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathLaneChange_double_f.time_elapsed));
figure
pathLaneChange_double_f.plot(); 



pathPoseLaneChange_double_s=[0 0 0; 8 0 0; ... 
    16 0 0; 24 4.62 30; 32 8 30; ...
    40 12.62 30; ...
    56 11.38 -30;64 8 -30; ...
    72 3.38 -30;80 0 -30;88 -4.62 -30; ...
    96 -8 -30;104 -12.62 -30; ...
    112 -16 0;120 -16 0; ...
    128 -12.62 30;136 -8 30; ...
    144 -4.62 30;152 0 0; ...
    160 0 0;168 0 0; ...
    176 0 0;184 0 0; ...
    192 0 0];
scaleMatrix = ones(size(pathPoseLaneChange_double_s));
scaleMatrix(:,2) = 0.15;
pathPoseLaneChange_double_s = pathPoseLaneChange_double_s.*scaleMatrix;
pathLaneChange_double_s = cidiPath(pathPoseLaneChange_double_s,8, 8, 8, 1000);
disp("Total Distance (m):")
disp(sum(pathLaneChange_double_s.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathLaneChange_double_s.time_elapsed));
figure
pathLaneChange_double_s.plot(); 

pathPoseFigureEight = [0 0 0; 100 0 0; 200 0 90; ...
    261.42 79.98 0; ...
    361.42 0 -45;461.42 -79.98 0; ...
    522.84 0 90;461.42 79.98 180; ...
    361.42 0 -135; ...
    261.42 -79.98 180;200 0 90];
pathFigureEight = cidiPath(pathPoseFigureEight, 10, 10, 10, 1000);
disp("Total Distance (m):")
disp(sum(pathFigureEight.distance_elapsed));
disp("Total Time (t):")
disp(sum(pathFigureEight.time_elapsed));
figure
pathFigureEight.plot();

pathPoseCircle_small_slow = [0 0 0];
prepath = 20;
radius = 100;
t = 0;
for i=1:50
    t = 3*pi/2 + (i-1)*(2*pi/50);
    [x,y] = pol2cart(t, radius);
    pathPoseCircle_small_slow = ...
        [pathPoseCircle_small_slow; prepath+x y+radius rad2deg(t)];
end
    
pathCircle_small_slow=cidiPath(pathPoseCircle_small_slow,8, 8, 8, 1000);
disp("Total Distance (m):")
disp(sum(pathCircle_small_slow.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathCircle_small_slow.time_elapsed));
figure
pathCircle_small_slow.plot();

pathPoseCircle_large_fast = [0 0 0];
prepath = 40;
radius = 100;
t = 0;
for i=1:50
    t = 3*pi/2 + (i-1)*(2*pi/50);
    [x,y] = pol2cart(t, radius);
    pathPoseCircle_large_fast = ...
        [pathPoseCircle_large_fast; prepath+x y+radius rad2deg(t)];
end
%pathPoseCircle_large_fast=[0 0 0;0 0 135;-29.3 70.7 90;0 141.4 45;141.4 141.4 -45;170.7 70.7 -90;141.4 0 -135;70.7 -29.3 -180;0 0 135];
pathCircle_large_fast=cidiPath(pathPoseCircle_large_fast,30, 30, 30, 1000);
disp("Total Distance (m):")
disp(sum(pathCircle_large_fast.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathCircle_large_fast.time_elapsed));
figure
pathCircle_large_fast.plot();

pathPoseGentleHighwayCurve=[0 0 0;20 1 0;60 2 45; ...
    160 5 36.135; ...
    259.81 9 30;289.81 14 20;319.81 17 0;349.81 20 0;379.81 25 0];
pathGentleHighwayCurve=cidiPath(pathPoseGentleHighwayCurve,30, 30, 30, 1000);
disp("Total Distance (m):")
disp(sum(pathGentleHighwayCurve.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathGentleHighwayCurve.time_elapsed));
figure
pathGentleHighwayCurve.plot()

pathPoseSpiral=[0 0 0;8.33 14.43 150;9.16 38.86 -150;-20.83 58.88 -120;-50 0 -90;-33.33 -57.74 -30;0 -75 0;80 -30 90;58.33 101.04 150;-66.67,115.47,-150;-83.33 -144.34 -30;0 -188.5 0;71.7 -188.8 30];
pathSpiral=cidiPath(pathPoseSpiral,17, 17, 17, 1000);
disp("Total Distance (m):")
disp(sum(pathSpiral.distance_elapsed));
disp("Total Time (t)：")
disp(sum(pathSpiral.time_elapsed));
figure
pathSpiral.plot()

%pathLaneChange.plot
%pathLaneChange_double_f.plot
%pathLaneChange_double_s.plot
%pathFigureEight.plot
%pathCircle_small_slow.plot
%pathCircle_large_fast.plot
%pathGentleHighwayCurve.plot

% pathLaneChange10.plot

% % TODO: Single Lane Change: Fast

% % TODO: Double Lane Change: Slow

% % TODO: Double Lane Change: Fast

% % TODO: Figure Eight

% % TODO: Circle: Small Slow

% % TODO: Circle: Large Fast

% % TODO: Gentle Highway Curve

% % TODO: Spiral: medium speed
