function [lat_output,heading_output] = Interpolate(v)
    heading_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    heading_err_scheduler_ratio = [0.8 0.6 0.4 0.2 0.1];%[1.0 0.6 0.4 0.2 0.1]
    
    lat_err_scheduler_v = [2.0 8.0 12.0 20.0 25.0];%[4.0 8.0 12.0 20.0 25.0]
    lat_err_scheduler_ratio = [0.8 0.6 0.2 0.1 0.05];%[1.0 0.6 0.2 0.1 0.05]
    
    lat_output = interp1(lat_err_scheduler_v,lat_err_scheduler_ratio,v,'spline');
    heading_output = interp1(heading_err_scheduler_v,heading_err_scheduler_ratio,v,'spline');

end