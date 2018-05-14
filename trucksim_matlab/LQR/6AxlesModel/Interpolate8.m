function [truck_lat_output,truck_heading_output, ...
    trailer_lat_output, trailer_heading_output] = Interpolate8(v)
    truck_heading_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    truck_heading_err_scheduler_ratio = [0.8 0.6 0.4 0.2 0.1];%[1.0 0.6 0.4 0.2 0.1]
    
    truck_lat_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    truck_lat_err_scheduler_ratio = [0.8 0.6 0.2 0.1 0.05];%[1.0 0.6 0.2 0.1 0.05]
    
    trailer_heading_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    trailer_heading_err_scheduler_ratio = [0.8 0.6 0.4 0.2 0.1];%[1.0 0.6 0.4 0.2 0.1]
    
    trailer_lat_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    trailer_lat_err_scheduler_ratio = [0.8 0.6 0.2 0.1 0.05];%[1.0 0.6 0.2 0.1 0.05]
    
    truck_lat_output = interp1(truck_lat_err_scheduler_v,truck_lat_err_scheduler_ratio,v,'spline');
    truck_heading_output = interp1(truck_heading_err_scheduler_v,truck_heading_err_scheduler_ratio,v,'spline');
    trailer_lat_output = interp1(trailer_lat_err_scheduler_v,trailer_lat_err_scheduler_ratio,v,'spline');
    trailer_heading_output = interp1(trailer_heading_err_scheduler_v,trailer_heading_err_scheduler_ratio,v,'spline');

end