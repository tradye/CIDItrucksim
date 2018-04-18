%% truck_LQRController S-function 20180326 xuhao
% states£ºlateral_error£¬lateral_error_rate£¬heading_error£¬heading_error_rate
% input£ºx,y,theta(heading),linear_v,angular_v,trailer_x,trailer_y,trailer_heading,trailer_heading_rate
% output£ºsteer_angle,v(v is constant)

function [sys,x0,str,ts] = truck_LQRController(t,x,u,flag)

switch flag,
 case 0
    [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
    sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
    sys = mdlOutputs(t,x,u); % Calculate outputs
  
%  case 9
%     sys = mdlTerminate(t,x,u);% terminate simulation
 
 case {1,4,9} % Unused flags
    sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end

function [sys,x0,str,ts] = mdlInitializeSizes
    sizes =simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates   = 4;
    sizes.NumOutputs      = 2;
    sizes.NumInputs         = 9; % x,y,heading,linear_v£¬angular_v,trailer_x,trailer_y,trailer_heading,trailer_heading_rate
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0 =[0;0;0;0];   

    % Initialize the discrete states.
    str = [];             % Set str to an empty matrix.
    ts  = [0.01 0];       % sample time: [period, offset]
    %End of mdlInitializeSizes
function sys = mdlUpdates(t,x,u)
    sys = x;
    %end of mdlUpdates
    
function sys = mdlOutputs(t,x,u)
    Nu = 2;%number of outputs
    Nx = 4;%number of states
    basic_state_size = 4;
    global target_points;
    global t_sim;
%     global num_i;
    
%     persistent truck_x_;
%     persistent truck_y_;
%     persistent truck_heading_;
%     persistent truck_lateral_err;
%     persistent truck_heading_err;
%     
%     persistent truck_state_;
    persistent time_sequence;
    persistent path_index;
    persistent path_x;
    persistent path_y;
    persistent path_heading;
    persistent steer_angle_;
    persistent truck_x_;
    persistent truck_y_;
    persistent truck_heading_;
    persistent truck_lateral_err;
    persistent truck_heading_err;
    persistent trailer_x_;
    persistent trailer_y_;
    persistent trailer_heading_;
    persistent trailer_heading_rate;
    
    num_i = calculate();
   
    pi = 3.1415926;
    t_s = 0.01;
    cf_ = 155494.663;
    cr_ = 155494.663;                            
    wheelbase_ = 4.750;                           % wheelbase
    mass_ = 8800.0;                                 % mass of the vehicle
    lf_ = 2.046;                                         % distance from front wheel center to COM
    lr_ = 1.84;                                           % distance from rear wheel center to COM
    iz_ = lf_^2 * 4400 + lr_^2 * 4400;      % rotational inertia
    steer_transmission_ratio_ = 20.3;        % the ratio between the turn of the steering wheel and the turn of the wheels
    steer_single_direction_max_degree_ = 800.0; % the maximum turn of steer
    max_lat_acc_ = 5.0;                           % limit steering to maximum theoretical lateral acceleration
    Vx = 0.0;
    lqr_max_iteration_ = 800;%150
    lqr_eps_ = 0.01;
    
    tic
%% define variables
    output =zeros(Nu,1);previousOutput =zeros(Nu,1);
    lateral_error_ = 0.0;heading_error_ = 0.0;
    lateral_error_rate = 0.0;heading_error_rate = 0.0;
    previous_lateral_error_ = 0.0;
    previous_heading_error_ = 0.0;
    truck_x = u(1);
    truck_y = u(2);
    truck_heading = u(3) * pi / 180;   %  the unit of heading in trucksim is deg, transform to rad
    truck_linear_v = u(4) / 3.6;           %  the unit of speed in trucksim is km/h, transform to m/s
    truck_angular_v = u(5) * pi / 180; %  the unit of angular in trucksim is deg/s, transform to rad/s
    trailer_x = u(6);
    trailer_y = u(7);
    trailer_heading = u(8) * pi / 180;   %  the unit of heading in trucksim is deg, transform to rad
    trailer_angular_v = u(9) * pi / 180; %  the unit of angular in trucksim is deg/s, transform to rad/s
    
    fprintf('num_i = %f \n',num_i); 
%     fprintf('trailer_x = %f \n',trailer_x); 
%     fprintf('trailer_y = %f \n',trailer_y); 
%     fprintf('trailer_heading = %f \n',trailer_heading); 
%     fprintf('trailer_angular_v = %f \n',trailer_angular_v); 

%     truck_x_(num_i,1) = truck_x;
%     truck_y_(num_i,1) = truck_y;
%     truck_heading_(num_i,1) = truck_heading;
% 
%     [fid , msg] = fopen('D:\CIDI\codes\matlab\MPC_LQR\truck_x.txt','wt');
%     if fid == -1
%         disp(msg);
%     else
%         fprintf(fid, '%3.6f  \n' ,truck_x_);
%         fclose(fid);
%     end
%     [fid , msg] = fopen('D:\CIDI\codes\matlab\MPC_LQR\truck_y.txt','wt');
%     if fid == -1
%         disp(msg);
%     else
%         fprintf(fid, '%3.6f  \n' ,truck_y_);
%         fclose(fid);
%     end
%     [fid , msg] = fopen('D:\CIDI\codes\matlab\MPC_LQR\truck_heading.txt','wt');
%     if fid == -1
%         disp(msg);
%     else
%         fprintf(fid, '%3.6f  \n' ,truck_heading_);
%         fclose(fid);
%     end
    
    target_points = readPlanPoints();
    
    matrix_A =zeros(basic_state_size,basic_state_size);
    matrix_Ad =zeros(basic_state_size,basic_state_size); %vehicle state matrix (discrete-time)
    matrix_B =zeros(basic_state_size,1);
    matrix_Bd =zeros(basic_state_size,1);                       % control matrix (discrete-time)
    matrix_K = zeros(1,basic_state_size);
    matrix_Q = zeros(basic_state_size,basic_state_size);
    matrix_R = eye(1,1);
    matrix_state = zeros(basic_state_size,1);
    %% matrix A init
    matrix_A(1,2) = 1.0;
    matrix_A(2,3) = (cf_ + cr_) / mass_;
    matrix_A(3,4) = 1.0;
    matrix_A(4,3) = (lf_ * cf_ - lr_ * cr_) / iz_;
    
    matrix_A(2,2) = -(cf_ + cr_) / mass_;
    matrix_A(2,4) = (lr_ * cr_ - lf_ * cf_) / mass_; 
    matrix_A(3,4) =  1.0;
    matrix_A(4,2) =  (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_A(4,4) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
    
    if(truck_linear_v < 0.2)   %
        Vx = 0.2;
    else
        Vx = truck_linear_v;
    end
    
    matrix_A(2,2) = matrix_A(2,2) / Vx;
    matrix_A(2,4) = matrix_A(2,4) / Vx;
    matrix_A(4,2) = matrix_A(4,2) / Vx;
    matrix_A(4,4) = matrix_A(4,4) / Vx;
    
    %% matirxB init
    matrix_B(2,1) =  cf_ / mass_;
    matrix_B(4,1) = lf_ * cf_ / iz_;
    
    matrix_Q(1,1) = 0.78245; %  0.01145 0.58245
    matrix_Q(2,2) = 0.000;
    matrix_Q(3,3) = 4.0000; %  0.0120  4.0000
    matrix_Q(4,4) = 0.0000;
    
    %% matrix_state init
    
%     [com_x,com_y] =ComputeCOMPosition(lr_,0,truck_x,truck_y,0);%compute COG based on truck position 
    
    [lateral_error_,lateral_error_rate,heading_error_,heading_error_rate,index_min]= ...
        ComputeLateralErrors(truck_x,truck_y,truck_heading,truck_linear_v,truck_angular_v);% 
    
    if(heading_error_  > pi)
        heading_error_ = heading_error_ - 2*pi;
    elseif (heading_error_  < -pi)
        heading_error_ = heading_error_ + 2*pi;
    else
        heading_error_ = heading_error_;
    end
    matrix_state(1,1) = lateral_error_;
    matrix_state(2,1) = lateral_error_rate;
    matrix_state(3,1) = heading_error_;
    matrix_state(4,1) = heading_error_rate;
    
%     truck_lateral_err(num_i,1) = lateral_error_;
%     truck_heading_err(num_i,1) = heading_error_;
    
    
%     for i = 1:1:size(truck_state_tmp , 2)
%         truck_state_(num_i , i) = truck_state_tmp(i);
%     end
    
%     [fid , msg] = fopen('D:\CIDI\codes\matlab\MPC_LQR\truck_lateralerr.txt','wt');
%     if fid == -1
%         disp(msg);
%     else
%         fprintf(fid, '%3.6f  \n' ,truck_lateral_err);
%         fclose(fid);
%     end
%     [fid , msg] = fopen('D:\CIDI\codes\matlab\MPC_LQR\truck_headingerr.txt','wt');
%     if fid == -1
%         disp(msg);
%     else
%         fprintf(fid, '%3.6f  \n' ,truck_heading_err);
%         fclose(fid);
%     end
    
        
    %% discretize matrix
    matrix_I = eye(basic_state_size,basic_state_size);
    matrix_Ad = (matrix_I + 0.5 * t_s * matrix_A) * inv(matrix_I - 0.5 * t_s * matrix_A);
    matrix_Bd = matrix_B * t_s;
    %% solve LQR problem
   
    [ratio_lat,ratio_heading] = Interpolate(truck_linear_v);
    matrix_Q(1,1) = ratio_lat *  matrix_Q(1,1);
    matrix_Q(3,3) = ratio_heading *  matrix_Q(3,3);
    matrix_K = SolveLQRProblem(matrix_Ad,matrix_Bd,matrix_Q,matrix_R,lqr_eps_,lqr_max_iteration_);
    steer_angle_feedback = -(matrix_K * matrix_state) * 180 / pi * steer_transmission_ratio_;
    
    
    %%   compute feedforward angle
    kv =  lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
    steer_angle_feedforwardterm =(wheelbase_ * target_points.curvature(index_min) + ...
                                                      kv * truck_linear_v * truck_linear_v * target_points.curvature(index_min) - ...
                                                       matrix_K(1, 3) *(lr_ * target_points.curvature(index_min) - ...
                                                       lf_ * mass_ * truck_linear_v * truck_linear_v * target_points.curvature(index_min) ...
                                                       / 2 / cr_ / wheelbase_)) *180 / pi * steer_transmission_ratio_ ;

    steer_angle = steer_angle_feedback + steer_angle_feedforwardterm;    
    %% limit output
    if steer_angle >= steer_single_direction_max_degree_
        steer_angle = steer_single_direction_max_degree_;
    elseif steer_angle <= -steer_single_direction_max_degree_
        steer_angle = -steer_single_direction_max_degree_;
    end
    %% s-function output
    truck_state_tmp = [num_i index_min target_points.x(index_min)  target_points.y(index_min) target_points.theta(index_min) ...
                                  steer_angle truck_x truck_y truck_heading lateral_error_ heading_error_ trailer_x trailer_y trailer_heading trailer_angular_v];
                              
    time_sequence(num_i,1) = truck_state_tmp(1);
    path_index(num_i,1) = truck_state_tmp(2);
    path_x(num_i,1) = truck_state_tmp(3);
    path_y(num_i,1) = truck_state_tmp(4);
    path_heading(num_i,1) = truck_state_tmp(5);
    steer_angle_(num_i,1) = truck_state_tmp(6);
    truck_x_(num_i,1) = truck_state_tmp(7);
    truck_y_(num_i,1) = truck_state_tmp(8);
    truck_heading_(num_i,1) = truck_state_tmp(9);
    truck_lateral_err(num_i,1) = truck_state_tmp(10);
    truck_heading_err(num_i,1) = truck_state_tmp(11);
    trailer_x_(num_i,1) = truck_state_tmp(12);
    trailer_y_(num_i,1) = truck_state_tmp(13);
    trailer_heading_(num_i,1) = truck_state_tmp(14);
    trailer_heading_rate(num_i,1) = truck_state_tmp(15);
    
    output(1,1) = steer_angle ;                         %unit deg
    output(2,1) = target_points.v(index_min) ;  %unit m/s
    fprintf('index_min = %f \n',index_min);
    t_sim = 34 ;%51.5  30
    if (num_i == (t_sim * 100+1) || ...
            sqrt( (truck_x - target_points.x(size(target_points.x , 1)))^2 + (truck_y - target_points.y(size(target_points.y , 1))) ^2 ) < 0.02 )
        plot_plot(truck_x_,truck_y_,target_points.x,target_points.y,truck_lateral_err,truck_heading_err);
        T = table(time_sequence,path_index, path_x, path_y, path_heading, steer_angle_,...
                   truck_x_, truck_y_, truck_heading_, truck_lateral_err, truck_heading_err ,...
                   trailer_x_ ,trailer_y_ , trailer_heading_ ,  trailer_heading_rate);
        writetable(T,'truck_state_.csv');
        
    end
    
   
    
    previousOutput(1,1) = output(1,1);
    previousOutput(2,1) = output(2,1);  
    sys = output;
     %% reach the terminal?
%      if sqrt( (truck_x - target_points.x(size(target_points.x) , 1))^2 + (truck_y - target_points.y(size(target_points.y) , 1)) ^2 ) < 0.02
%          sys = [0;0];
%      else
%          sys = output;
%      end
%     
%     sys = mdlTerminate(t,x,u);% terminate simulation
%     sys = [];
%     
    toc
    
    





