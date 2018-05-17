function [trailer_points ] = generateTrailerPoints( truck_points, ...
                                                    method, ...
                                                    hF, ...
                                                    hR)
% hR is typically defined as truck COG to hitch distance
% hF is typically defined as trailer COG to hitch distance
    if method == "kinematic"
        num_points = size(truck_points.x, 1);
        array_index = linspace(2, num_points, num_points - 2 + 1);
%         array_index = linspace(1, num_points, num_points);
        array_dt = [ 0, ...
            arrayfun(@(i) funcCalculateElapsedTimeFromPoints( ...
            truck_points.x(i), truck_points.y(i), ...
            truck_points.x(i-1), truck_points.y(i-1), ...
            truck_points.v(i) ...
            ), ...
            array_index)];
%         
        value1 = [truck_points.x(1) - hF - hR]; % x
        value2 = [0]; % y
        value3 = [0]; % theta
        value4 = [0]; % v
        value5 = [0]; % curvature
        value6 = [0]; % curvature_change_rate
        
        for i=2:num_points
            x0 = truck_points.x(i);
            y0 = truck_points.y(i);
            v0 = truck_points.v(i);
            h0_prev = truck_points.theta(i-1);
            h1_prev = value3(i-1);
            h0 = truck_points.theta(i);
            h1 = wrapToPi(h0 + v0/hR * sin(h0_prev - h1_prev)*array_dt(i));
            x1 = x0 - hF*cos(h0) - hR*cos(h1);
            y1 = y0 - hF*sin(h0) - hR*sin(h1);
            dh0 = value5(i-1);
            dh1 = (wrapToPi((h1-h1_prev))/array_dt(i))/v0;
            ddh1 = (dh1-dh0)/array_dt(i);
            
            value1 = [value1; x1];
            value2 = [value2; y1];
            value3 = [value3; h1];
            value4 = [value4; v0];
            value5 = [value5; dh1];
            value6 = [value6; ddh1];
           
        end % for i=2:num_points
        
        field1 = 'x';
        field2 = 'y';
        field3 = 'theta';
        field4 = 'v';
        field5 = 'curvature';   
        field6 = 'curvature_change_rate';

        trailer_points = struct(field1,value1, ...
                                field2,value2, ...
                                field3,value3, ...
                                field4,value4, ...
                                field5,value5, ...
                                field6,value6);    

    else
        trailer_points = truck_points;    
    end % method == "kinematic"
end % generateTrailerPoints

function elapsed_time = funcCalculateElapsedTimeFromPoints(x1,y1,x0,y0,v1)
    elapsed_time = sqrt((x1-x0)^2 + (y1-y0)^2 )/v1;
end % function