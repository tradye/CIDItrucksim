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

% clear;

% % TODO: Straight Path
pathPoses = [0 0 0; 20 0 0; 40 0 0; 60 0 0; 80 0 0];
pathPoses = [pathPoses; 100 0 0; 120 0 0; 140 0 0; 160 0 0];
pathPoses = [pathPoses; 180 0 0; 200 0 0];
pathStraightSlow = cidiPath(pathPoses, 10, 10, 10, 1000);


% % TODO: Straight Diagonal Path
pathPoses = [0 0 45; 20 10 45; 40 20 0; 60 30 45; 80 40 45];
pathPoses = [pathPoses; 100 50 45; 120 60 45; 140 70 45; 160 80 45];
pathPoses = [pathPoses; 180 90 45; 200 100 45];

pathStraightFast = cidiPath(pathPoses, 15, 15, 15, 1000);
disp("Total Distance (m):")
disp(sum(pathStraightFast.distance_elapsed));
disp("Total Time (t):")
disp(sum(pathStraightFast.time_elapsed));

% % Example: Single Lane Change: Slow
pathPoses = [0 0 0; 20 0 0; 40 0 0; 60 1 45; 80 2 45];
pathPoses = [pathPoses; 100 3 45; 120 3 0; 140 3 0; 160 3 0];
pathPoses = [pathPoses; 180 3 0; 200 3 0];

pathLCSlow = cidiPath(pathPoses, 15, 15, 15, 1000);
disp("Total Distance (m):")
disp(sum(pathLCSlow.distance_elapsed));
disp("Total Time (t):")
disp(sum(pathLCSlow.time_elapsed));
% pathLaneChange10.plot

% % TODO: Single Lane Change: Fast
pathPoses = [0 0 0; 40 0 0; 80 0 0; 120 1 45; 160 2 45];
pathPoses = [pathPoses; 200 3 45; 240 4 0; 280 4 0; 320 4 0];
pathPoses = [pathPoses; 360 4 0; 400 4 0];

pathLCFast = cidiPath(pathPoses, 30, 30, 30, 1000);
disp("Total Distance (m):")
disp(sum(pathLCFast.distance_elapsed));
disp("Total Time (t):")
disp(sum(pathLCFast.time_elapsed));
% % TODO: Double Lane Change: Slow

% % TODO: Double Lane Change: Fast

% % TODO: Figure Eight

% % TODO: Circle: Small Slow

% % TODO: Circle: Large Fast

% % TODO: Gentle Highway Curve

% % TODO: Spiral: medium speed

