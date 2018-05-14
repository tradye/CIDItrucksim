%% truck_MPCController S-function 20180502 xuhao
% states£ºlateral_error£¬lateral_error_rate£¬heading_error£¬heading_error_rate,station_error,speed_error
% input£ºx,y,theta(heading),linear_v,angular_v
% output£ºsteer_angle,v(v is constant)

function [sys,x0,str,ts] = truck_MPCController(t,x,u,flag)

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end

function [sys,x0,str,ts] = mdlInitializeSizes
    sizes =simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates   = 6;% LQR 4
    sizes.NumOutputs      = 2;
    sizes.NumInputs         = 9; % x,y,heading,linear_v£¬angular_v
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0 =[0;0;0;0;0;0];   

    % Initialize the discrete states.
    str = [];             % Set str to an empty matrix.
    ts  = [0.01 0];       % sample time: [period, offset]
    %End of mdlInitializeSizes
function sys = mdlUpdates(t,x,u)
    sys = x;
    %end of mdlUpdates
    
function sys = mdlOutputs(t,x,u)
    Nu = 2;%number of outputs
    Nx = 6;%number of states
    Np = 10;%number of states
    basic_state_size = 6; % in LQR is 4
    global target_points;
    persistent truck_path_points;
    global trailer_points;
    persistent trailer_path_points;
    global t_sim;     
    
    persistent time_sequence_;
    persistent path_index_;
    persistent path_x_;
    persistent path_y_;
    persistent path_heading_;
    persistent path_curvature_;
    
    persistent steer_angle_;
    
    persistent truck_x_;
    persistent truck_y_;    
    persistent truck_heading_;
    persistent truck_heading_rate_;
    
    persistent truck_lateral_err_;
    persistent truck_heading_err_;
    
    persistent trailer_path_x_;
    persistent trailer_path_y_;
    persistent trailer_path_heading_;
    persistent trailer_path_curvature_;   
    
    persistent trailer_x_;
    persistent trailer_y_;
    persistent trailer_heading_;
    persistent trailer_heading_rate_;
    
    persistent trailer_lateral_err_;
    persistent trailer_heading_err_;
    
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
    mpc_max_iteration_ = 800;%150
    mpc_eps_ = 0.01;
    max_deceleration = -6;
    max_acceleration = 2;
    CONST_hF = 4.24;
    CONST_hR = 3.4;
    tic

%% define variables
    output =zeros(Nu,1);previousOutput =zeros(Nu,1);
%     lateral_error_ = 0.0;heading_error_ = 0.0;
%     lateral_error_rate = 0.0;heading_error_rate = 0.0;
%     station_error_ = 0.0;
%     speed_error_ = 0.0;
%     previous_lateral_error_ = 0.0;
%     previous_heading_error_ = 0.0;
    truck_x = u(1);
    truck_y = u(2);
    truck_heading = u(3) * pi / 180;   %  the unit of heading in trucksim is deg, transform to rad
    truck_linear_v = u(4) / 3.6;           %  the unit of speed in trucksim is km/h, transform to m/s
    truck_angular_v = u(5) * pi / 180; %  the unit of angular in trucksim is deg/s, transform to rad/s
    
    trailer_x = u(6);
    trailer_y = u(7);
    trailer_heading = u(8) * pi/180;
    trailer_angular_v = u(9) * pi / 180;
    
    fprintf('num_i = %f \n',num_i); 

    truck_x = truck_x - CONST_hF * cos(truck_heading);
    truck_y = truck_y - CONST_hF * sin(truck_heading);
    
    trailer_x = trailer_x - CONST_hR * cos(trailer_heading);
    trailer_y = trailer_y - CONST_hR * sin(trailer_heading);
    
    tic
    if(num_i == 1)
        truck_path_points = readPlanPoints();
        for i = 1:1:size(truck_path_points.x )
            truck_path_points.x(i) =  truck_path_points.x(i) - 4.25;
%             truck_path_points.v(i) =  truck_path_points.v(i) - 12;
        end
        target_points = truck_path_points;
        trailer_path_points = generateTrailerPoints(truck_path_points, "kinematic",CONST_hF,CONST_hR);
        trailer_points = trailer_path_points;  
    end




    matrix_A =zeros(Nx,Nx);
    matrix_Ad =zeros(Nx,Nx); %vehicle state matrix (discrete-time)
    matrix_B =zeros(Nx,Nu);
    matrix_Bd =zeros(Nx,Nu); % control matrix (discrete-time)
    matrix_C =zeros(Nx,1);
    matrix_Cd =zeros(Nx,1);   % control matrix (discrete-time)
    
    matrix_K = zeros(1,Nx);
    matrix_Q = zeros(Nx,Nx);
    matrix_R = eye(Nu,Nu);
    matrix_state = zeros(Nx,1);
    matrix_control = zeros(Nu,1);
    matrix_reference_state = zeros(Nx,1);
    
    matrix_lower_bound = zeros(Nu,1);
    matrix_upper_bound = zeros(Nu,1);
    reference = cell(Np,1);
    control = cell(Np,1);
    
    for i = 1:1:Np
        reference{i} = matrix_reference_state;
        control{i,1} = matrix_control;
    end
    %% matrix A init
    matrix_A(1,2) = 1.0;
    matrix_A(2,3) = (cf_ + cr_) / mass_;
    matrix_A(3,4) = 1.0;
    matrix_A(4,3) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_A(5,5) =1.0; %new added for longitudinal control in MPC
    matrix_A(6,6) = 0.0;%new added for longitudinal control in MPC
    
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
    
    %% matirxB/C/Q init
    matrix_B(2,1) =  cf_ / mass_;
    matrix_B(4,1) = lf_ * cf_ / iz_;
    matrix_B(5,2) =  0.0;%new added for longitudinal control in MPC
    matrix_B(6,2) = 1.0; %new added for longitudinal control in MPC
    
    matrix_C(2,1) =  (lr_ * cr_ - lf_ * cf_) / mass_ / Vx - Vx;
    matrix_C(4,1) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / Vx;
    
    matrix_Q(1,1) = 1.5; %  0.01145 0.58245
    matrix_Q(2,2) = 0.000;
    matrix_Q(3,3) = 5.0000; %  0.0120  4.0000
    matrix_Q(4,4) = 0.0000;

    max_steer_angle = deg2rad(steer_single_direction_max_degree_);
    matrix_lower_bound(1,1) = -max_steer_angle;
    matrix_lower_bound(2,1) = max_deceleration;
    matrix_upper_bound(1,1) = max_steer_angle;
    matrix_upper_bound(2,1) = max_acceleration;
    
    %% matrix_state init
    
%     [com_x,com_y] =ComputeCOMPosition(lr_,0,truck_x,truck_y,0);%compute COG based on truck position 
[truck_lateral_err,truck_lateral_err_rate,truck_heading_err,truck_heading_err_rate, ...
    trailer_lateral_err, trailer_lateral_err_rate, ...
    trailer_heading_err, trailer_heading_err_rate,station_err,speed_err, index_min] = ...
    ComputeErrors(truck_x,truck_y,trailer_x,trailer_y, ...
    truck_heading, trailer_heading,truck_linear_v,truck_angular_v,trailer_angular_v);

    
    if(truck_heading_err  > pi)
        truck_heading_err = truck_heading_err - 2*pi;
    elseif (truck_heading_err  < -pi)
        truck_heading_err = truck_heading_err + 2*pi;
    else
        truck_heading_err = truck_heading_err;
    end
    
    if(trailer_heading_err  > pi)
        trailer_heading_err = trailer_heading_err - 2*pi;
    elseif (trailer_heading_err  < -pi)
        trailer_heading_err = trailer_heading_err + 2*pi;
    else
        trailer_heading_err = trailer_heading_err;
    end
    
    matrix_state(1,1) = truck_lateral_err;
    matrix_state(2,1) = truck_lateral_err_rate;
    matrix_state(3,1) = truck_heading_err;
    matrix_state(4,1) = truck_heading_err_rate;
    matrix_state(5,1) = station_err;
    matrix_state(6,1) = speed_err;

    %% discretize matrix
    matrix_I = eye(basic_state_size,basic_state_size);
    matrix_Ad = (matrix_I + 0.5 * t_s * matrix_A) * inv(matrix_I - 0.5 * t_s * matrix_A);
    matrix_Bd = matrix_B * t_s;
    matrix_Cd = matrix_C * truck_heading_err_rate * t_s;
    %% solve MPC problem

    [ratio_lat,ratio_heading] = Interpolate(truck_linear_v);
    matrix_Q(1,1) = ratio_lat *  matrix_Q(1,1);
    matrix_Q(3,3) = ratio_heading *  matrix_Q(3,3);

%     matrix_K = SolveLQRProblem(matrix_Ad,matrix_Bd,matrix_Q,matrix_R,lqr_eps_,lqr_max_iteration_);
    control = SolveMPCProblem(matrix_Ad,matrix_Bd,matrix_Cd,matrix_Q,matrix_R,matrix_lower_bound,matrix_upper_bound,...
                                                 reference,matrix_state,control);
    steer_angle_feedback = control{1}(1,1)  * 180 / pi * steer_transmission_ratio_;
    
    %%   compute feedforward angle
%     kv =  lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
%     steer_angle_feedforwardterm =(wheelbase_ * target_points.curvature(index_min) + ...
%                                                       kv * truck_linear_v * truck_linear_v * target_points.curvature(index_min) - ...
%                                                        matrix_K(1, 3) *(lr_ * target_points.curvature(index_min) - ...
%                                                        lf_ * mass_ * truck_linear_v * truck_linear_v * target_points.curvature(index_min) ...
%                                                        / 2 / cr_ / wheelbase_)) *180 / pi * steer_transmission_ratio_ ;
    steer_angle_feedforwardterm = wheelbase_ * target_points.curvature(index_min);
    steer_angle = steer_angle_feedback + steer_angle_feedforwardterm;    
    %% limit output
    if steer_angle >= steer_single_direction_max_degree_
        steer_angle = steer_single_direction_max_degree_;
    elseif steer_angle <= -steer_single_direction_max_degree_
        steer_angle = -steer_single_direction_max_degree_;
    end
    %% s-function output
     truck_state_tmp = [ ...
        num_i, ... 
        index_min, ...
        target_points.x(index_min), ...
        target_points.y(index_min), ...
        target_points.theta(index_min), ... % 5
        target_points.curvature(index_min), ...
        steer_angle, ...
        truck_x, ...
        truck_y, ...
        truck_heading, ... % 5
        truck_angular_v, ...
        truck_lateral_err, ...
        truck_heading_err,...
        trailer_points.x(index_min), ...
        trailer_points.y(index_min), ...
        trailer_points.theta(index_min), ...
        trailer_points.curvature(index_min), ...
        trailer_x, ...
        trailer_y, ...
        trailer_heading, ...% 5
        trailer_angular_v, ...
        trailer_lateral_err, ...
        trailer_heading_err
        ];
                              
    time_sequence_(num_i,1) = truck_state_tmp(1);
    path_index_(num_i,1) = truck_state_tmp(2);
    
    path_x_(num_i,1) = truck_state_tmp(3);
    path_y_(num_i,1) = truck_state_tmp(4);
    path_heading_(num_i,1) = truck_state_tmp(5);
    path_curvature_(num_i,1) = truck_state_tmp(6);
    
    steer_angle_(num_i,1) = truck_state_tmp(7);
    
    truck_x_(num_i,1) = truck_state_tmp(8);
    truck_y_(num_i,1) = truck_state_tmp(9);
    truck_heading_(num_i,1) = truck_state_tmp(10);
    truck_heading_rate_(num_i,1) = truck_state_tmp(11);
    truck_lateral_err_(num_i,1) = truck_state_tmp(12);
    truck_heading_err_(num_i,1) = truck_state_tmp(13);
    
    trailer_path_x_(num_i,1) = truck_state_tmp(14);
    trailer_path_y_(num_i,1) = truck_state_tmp(15);
    trailer_path_heading_(num_i,1) = truck_state_tmp(16);
    trailer_path_curvature_(num_i,1) = truck_state_tmp(17);
    
    trailer_x_(num_i,1) = truck_state_tmp(18);
    trailer_y_(num_i,1) = truck_state_tmp(19);
    trailer_heading_(num_i,1) = truck_state_tmp(20);
    trailer_heading_rate_(num_i,1) = truck_state_tmp(21);
    trailer_lateral_err_(num_i,1) = truck_state_tmp(22);
    trailer_heading_err_(num_i,1) = truck_state_tmp(23);
   
    output(1,1) = steer_angle ;                         %unit deg
    output(2,1) = target_points.v(index_min) ;  %unit m/s
    fprintf('index_min = %f \n',index_min);
    t_sim = 30; %51.5  30
    
    distance_to_end_point = sqrt( (truck_x - target_points.x(size(target_points.x , 1)))^2 + (truck_y - target_points.y(size(target_points.y , 1))) ^2 );
    if (num_i == (t_sim * 100+1) || distance_to_end_point < 0.2 )
        plot_plot8(...
            truck_x_, ...
            truck_y_, ...
            target_points.x, ...
            target_points.y, ...
            truck_lateral_err_, ... % 5
            truck_heading_err_, ...
            trailer_x_, ...
            trailer_y_, ...
            trailer_points.x, ...
            trailer_points.y, ... % 5
            trailer_lateral_err_, ...
            trailer_heading_err_, ... 
            steer_angle_, ...
            num_i ...
            );
        T = table( ...
            time_sequence_, ...
            path_index_,  ...
            path_x_,  ...
            path_y_,  ...
            path_heading_,  ... % 5
            path_curvature_,  ...
            steer_angle_,...
            truck_x_,  ...
            truck_y_,  ...
            truck_heading_,  ... % 5
            truck_heading_rate_,  ...
            truck_lateral_err_,  ...
            truck_heading_err_ , ...
            trailer_path_x_,  ...
            trailer_path_y_,  ... % 5
            trailer_path_heading_,  ...
            trailer_path_curvature_,  ...
            trailer_x_, ...
            trailer_y_ ,  ...
            trailer_heading_ ,   ... % 5
            trailer_heading_rate_, ...
            trailer_lateral_err_, ...
            trailer_heading_err_ ...
            );
        writetable(T,'D:\cidi\installed_soft\trucksim\TruckSim2016.1_Data\truckStateFollowing_.csv');
        
    end
    
    previousOutput(1,1) = output(1,1);
    previousOutput(2,1) = output(2,1);  
    sys = output;
    
    toc
    
    





