function [lat_err,lat_err_rate,heading_err,heading_err_rate, ...
    trail_lat_err, trail_lat_err_rate, ...
    trail_heading_err, trail_heading_err_rate, i] = ...
    ComputeLateralErrors(xk,yk,xl,yl,hk,hl,linear_v_,avk,avl)

   global target_points;
   global trailer_points;
   i =  QueryNearestPointByPosition(xk,yk);
   
   dxk = xk - target_points.x(i);
   dyk = yk - target_points.y(i);
   d_hk = hk - target_points.theta(i);

   lat_err = dyk * cos(target_points.theta(i)) - dxk * sin(target_points.theta(i));
   lat_err_rate = linear_v_ * sin(d_hk);
   
   heading_err = d_hk;
   heading_err_rate = avk - target_points.curvature(i) * target_points.v(i);
   
   dxl = xl - trailer_points.x(i);
   dyl = yl - trailer_points.y(i);
   d_hl = hl - trailer_points.theta(i);
   
   trail_lat_err = dyl * cos(trailer_points.theta(i)) - dxl * sin(trailer_points.theta(i));
   trail_lat_err_rate = linear_v_ * sin(d_hl);
   
   trail_heading_err = d_hl;
   trail_heading_err_rate = avl - trailer_points.curvature(i) * trailer_points.v(i);      
end

