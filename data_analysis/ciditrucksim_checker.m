close all;
clear;

% CONSTANTS
% Active Directory Name
cd /home/husibo/CidiGit/CidiTruckModel/
% Sim Results Directory Name
cd ./ciditrucksim/sim_results/;
% File Prefix name
CONST_FILE_PREFIX = "Run_";
CONST_MAX_FILES = 100;

CONST_hF = 5.635;
CONST_hR = 0;




% Max of 100 files
data = {};
file_index = 1;

% Read all Run_ files in sim_results
pathlisting = dir(".");
for pathdir = pathlisting'    
    if pathdir.name ~= "." && pathdir.name ~= ".."        
        pathdirlisting = dir("./" + pathdir.name);
        for listing = pathdirlisting'            
            if ~isempty(strfind(listing.name,CONST_FILE_PREFIX))
                disp("Reading " + listing.name);
                data{file_index} = csvread("./" + pathdir.name + "/" + listing.name, 1, 0);
                file_index = file_index + 1;
            end % ~isempty
        end % for listing        
    end % if pathdir.name ~= "."
end

%i = 4;
for i=1:length(data)   
    M = data{i};
    T = M(:,1);
    XK = M(:,2);
    XL = M(:,3);
    YK = M(:,4);
    YL = M(:,5);
    DM = M(:,6);
    SW = M(:,7);
    HK = M(:,8)*pi/180;
    HL = M(:,9)*pi/180;
    AVK = M(:,10)*pi/180;
    AVL = M(:,11)*pi/180;
    n = size(M,1);
    
    [pred_trailer_x,pred_trailer_y] = ...
        arrayfun(@(x) calc_trailer_position(XK(x), YK(-x), ...
        HK(x), HL(x), CONST_hF, CONST_hR), ...
        linspace(1, n, n));
    [pred_av] = [0,arrayfun(@(x) rate(HK(x),HK(x-1),T(x)-T(x-1)), ...
        linspace(2,n,n-1))]*180/pi;
    
    C = [];
    C(:,1) = pred_trailer_x';
    C(:,2) = pred_trailer_y';
    C(:,3) = XL - pred_trailer_x';
    C(:,4) = YL - pred_trailer_y';
    C(:,5) = pred_av';
    
    data{i}(:,12) = pred_trailer_x';
    data{i}(:,13) = pred_trailer_y';
    data{i}(:,14) = C(:,3);
    data{i}(:,15) = C(:,4);
    data{i}(:,16) = C(:,5);
    % obj.angle = transpose(arrayfun(@(x) atan2(obj.data.dy(x),obj.data.dx(x)), ...
end
    

% Fields:
% Time,Xo      ,Xo_2    ,Yo      ,Yo_2    ,
% Steer_DM,Steer_SW,Yaw     ,Yaw_2   ,AVz     ,AVz_2 

function [x,y] = calc_trailer_position(truckX, ...
    truckY, truckheading, trailerheading, hF, hR)
    x = truckX - hF*cos(truckheading) - hR*cos(trailerheading);
    y = truckY - hF*sin(truckheading) - hR*sin(trailerheading);
end % calc_trailer_position

function val = calc_squared_distance(X0, Y0, X1, Y1)
    val = (X1-X0)^2 + (Y1-Y0)^2;
end % calc_squared_distance

function val = rate(x1, x0, dt)
    val = (x1-x0)/dt;
end % calc_rate

