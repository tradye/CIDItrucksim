function control_out = SolveCiDiMPCProblem(A,B,D1,D2,Q,R, ...
                                           h, ...
                                           init_state, ...
                                           truck_angular_v,...
                                           trailer_angular_v,...
                                           max_steering_angle)
    % Usage:
    % t_s 0.01;
    % matrix_Ad = truckmodel.Ad(t_s);
    % matrix_Bd = truckmodel.Cd(t_s);
    % matrix_D1 = truckmodel.D1(t_s,1);
    % matrix_D2 = truckmodel.D2(t_s,2);
    % % matrix_Q as defined
    % % matrix_R as defined
    % h = max(150, length(target_points.x) - index_min);
    % init_state = matrix_state;
    % arr_truck_angular_v = ...
    %    target_points.curvature(index_min:index_min+h-1).*target_points.v(index_min:index_min+h-1);
    % arr_trailer_angular_v = ...
    %    trailer_points.curvature(index_min:index_min+h-1).*trailer_points.v(index_min:index_min+h-1);
    % max_steering_angle = deg2rad(45);
    % control = SolveCiDiMPCProblem(matrix_Ad, matrix_Bd, matrix_D1, ...
    %   matrix_D2, matrix_Q, matrix_R, h, init_state, ...
    %   arr_truck_angular_v, arr_trailer_angular_v, ...
    %   max_steer_angle);
    % steer_angle_feedback = control * 180/ pi * steer_transmission_ratio_;
    
   
    QQ = kron(eye(h),Q);
    RR = kron(eye(h), R);
    I = eye(h);
    K = kron(eye(h), B);
     for i=2:h
         K = K + kron([zeros((i-1),h);I(1:end-i+1,:)], ...
             (A^i) * B);
     end % for
     MM = [];
%      MM_new = A*init_state + ...
%          D1*arr_truck_angular_v(1) + D2*arr_trailer_angular_v(1);
%      MM_new = A*init_state ;
     MM_new = A*init_state + ...
         D1*truck_angular_v + D2*trailer_angular_v;
     MM = [MM;MM_new];
     for i=2:h
%          MM_new = A*MM_new + ...
%          D1*arr_truck_angular_v(i) + D2*arr_trailer_angular_v(i);
%          MM_new = A*MM_new;
         MM_new = A*MM_new + ...
            D1*truck_angular_v + D2*trailer_angular_v;
         MM = [MM;MM_new];
     end
     H = K'*QQ*K+RR;
     QQQ = K'*QQ*MM;
     [L, ~] = chol(H,'lower');
     Linv = inv(L);
     opt = mpcqpsolverOptions;
     tmp1 =  [eye(h);-eye(h)];
     tmp2 = [-ones(h,1); -ones(h,1)]*max_steering_angle;
     control_out = mpcqpsolver(Linv, QQQ, ...
         tmp1, tmp2, ...
         [], zeros(0,1), false(size(zeros(2*h,1))), opt);     
end