function [lateral_err,lateral_err_rate,heading_err,heading_err_rate,i]= ComputeLateralErrors(x_,y_,theta_,linear_v_,angular_v_)

%  the input parameters are the real states of truck   
   global target_points;  % declare if u want using
   i =  QueryNearestPointByPosition(x_,y_);
   
   dx = x_ - target_points.x(i);
   dy = y_ - target_points.y(i);
   d_theta = theta_ - target_points.theta(i);

   lateral_err = dy * cos(target_points.theta(i)) - dx * sin(target_points.theta(i));
   lateral_err_rate = linear_v_ * sin(d_theta);
   
   heading_err = d_theta;
   heading_err_rate = angular_v_ - target_points.curvature(i) * target_points.v(i);
    
end