function [target_points] = readCidiPathPoints(input_cidipath)
% Read cidiPath and create a target_points object  
    
    


    field1 = 'x'; value1 = input_cidipath.x;
    field2 = 'y'; value2 = input_cidipath.y;
    field3 = 'theta'; value3 = input_cidipath.angle;
    field4 = 'v'; value4 = input_cidipath.speedProfile;
    field5 = 'curvature'; value5 = input_cidipath.angle_rate./input_cidipath.speedProfile;    
    field6 = 'curvature_change_rate'; value6 = ...
        [0; diff(input_cidipath.angle_rate)./input_cidipath.time_elapsed(2:end)];

    target_points = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);    

    

end % readCidiPathPoints