%% truck8_MPCController S-function 20180502 xuhao husibo
% states:truck_lateral_error,truck_lateral_error_rate,truck_heading_error,
%        truck_heading_error_rate,
%        trailer_lateral_error,trailer_lateral_error_rate,trailer_heading_error,
%        trailer_heading_error_rate
% input:x,y,theta(heading),linear_v,angular_v,trailer_x,trailer_y,trailer_heading,
%       trailer_heading_rate
% output:steer_angle,v(v is constant)

function [sys,x0,str,ts] = truck8_MPCController(t,x,u,flag)

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
    sizes.NumDiscStates   = 8;
    sizes.NumOutputs      = 2;
    sizes.NumInputs       = 9; % 
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes); 
    x0 =[0;0;0;0;0;0;0;0];   

    % Initialize the discrete states.
    str = [];             % Set str to an empty matrix.
    ts  = [0.01 0];       % sample time: [period, offset]
    %End of mdlInitializeSizes
function sys = mdlUpdates(t,x,u)
    sys = x;
    %end of mdlUpdates
    
function sys = mdlOutputs(t,x,u)

    Nu = 2;%number of outputs
    Nx = 8;%number of states
    basic_state_size = 8;
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
    wheelbase_ = 4.750;                           % wheelbase
    
    CONST_C1 = 155494*2;
    CONST_C2 = 155494*4;
    CONST_C3 = 155494*4;
    CONST_C4 = 155494*4;
    CONST_C5 = 155494*4;
    CONST_C6 = 155494*4;    
    
%     CONST_TRUCK_COG_FRAC = 0.25;
%     CONST_TRAIL_COG_FRAC = 0.15;
%     CONST_b1 = 3.9 * CONST_TRUCK_COG_FRAC;%distance from first axle to COG of truck
%     CONST_b2 = 3.9 * (1-CONST_TRUCK_COG_FRAC);%distance from  COG of truck to second axle 
%     CONST_b3 = CONST_b2+1.310;%distance from  COG of truck to third axle 
% 
%     CONST_b4 = 4.4*(1-CONST_TRAIL_COG_FRAC);%distance from  COG of trailer to forth axle 
%     CONST_b5 = CONST_b4+1.310;%distance from  COG of trailer to fifth axle
%     CONST_b6 = CONST_b5+1.310;%distance from  COG of trailer to sixth axle
    CONST_TRUCK_COG_FRAC = 0.25;
    CONST_TRAIL_COG_FRAC = 0.15;
    CONST_b1 = 1.385;%distance from first axle to COG of truck
    CONST_b2 = 3.615;%distance from  COG of truck to second axle 
    CONST_b3 = CONST_b2+1.270;%distance from  COG of truck to third axle 

    CONST_b4 = 3.3;%distance from  COG of trailer to forth axle 
    CONST_b5 = CONST_b4+1.2;%distance from  COG of trailer to fifth axle
    CONST_b6 = CONST_b5+1.2;%distance from  COG of trailer to sixth axle

    % Mass of each body
%     CONST_m1 = 8800;%
%     CONST_m2 = 20000;
    CONST_m1 = 6310;%
    LOADED = false;% true false
    if LOADED == true
        CONST_m2 = 5500 + 20582.76;%load status
    else
        CONST_m2 = 5500;%unload status
    end
    
    
    

    % Distance of hitch to COG for each body
%     CONST_hF = CONST_b2 + 0.075;
%     CONST_hR = 7.16-4.4*(1-CONST_TRAIL_COG_FRAC); % approximation of hitch-COG distance of trailer
    CONST_hF = 4.25; %distance from truck-COG to hitch
    CONST_hR = 3.4; % approximation of hitch-COG distance of trailer

    % Inertia moment of each body
    CONST_m1_f = CONST_m1*0.75;
    CONST_m1_r = CONST_m1*0.25;
    CONST_m2_f = CONST_m2*0.25;
    CONST_m2_r = CONST_m2*0.75; % assume the rear wheelbases carry 75% of the weight
%     CONST_I1 = CONST_m1_f*b1*b1 + CONST_m1_r*CONST_hF*CONST_hF;
    CONST_I1 = CONST_m1_f*CONST_b1*CONST_b1 + CONST_m1_r*CONST_hF*CONST_hF;
    CONST_I2 = CONST_m2_f*CONST_hR*CONST_hR + CONST_m2_r*CONST_b5*CONST_b5;

    truckmodel = TruckModel6Axle();
    truckmodel.set_stiffness(CONST_C1, CONST_C2, CONST_C3, CONST_C4, CONST_C5, CONST_C6); 
    truckmodel.set_axle_dist(CONST_b1, CONST_b2, CONST_b3, CONST_b4, CONST_b5, CONST_b6);
    truckmodel.set_hitch_dist(CONST_hF, CONST_hR);
    truckmodel.set_mass(CONST_m1, CONST_m2);
    truckmodel.set_inertia(CONST_I1, CONST_I2);
    
    steer_transmission_ratio_ = 20.3;        % the ratio between the turn of the steering wheel and the turn of the wheels
    steer_single_direction_max_degree_ = 800.0; % the maximum turn of steer
    
    max_lat_acc_ = 5.0;                           % limit steering to maximum theoretical lateral acceleration
    Vx = 0.0;
    lqr_max_iteration_ = 800;%150
    lqr_eps_ = 0.01;
    tic

%% define variables
    output =zeros(Nu,1);previousOutput =zeros(Nu,1);
%     lateral_error_ = 0.0;heading_error_ = 0.0;
%     lateral_error_rate = 0.0;heading_error_rate = 0.0;
%     previous_lateral_error_ = 0.0;
%     previous_heading_error_ = 0.0;
    truck_x = u(1)  ;
    truck_y = u(2);
    truck_heading = u(3) * pi / 180;   %  the unit of heading in trucksim is deg, transform to rad
    truck_linear_v = u(4) / 3.6;           %  the unit of speed in trucksim is km/h, transform to m/s
    truck_angular_v = u(5) * pi / 180; %  the unit of angular in trucksim is deg/s, transform to rad/s
    trailer_x = u(6);
    trailer_y = u(7);
    trailer_heading = u(8) * pi/180;
    trailer_angular_v = u(9) * pi / 180;
    
    truck_x = truck_x - CONST_hF * cos(truck_heading);
    truck_y = truck_y - CONST_hF * sin(truck_heading);
    
    trailer_x = trailer_x - CONST_hR * cos(trailer_heading);
    trailer_y = trailer_y - CONST_hR * sin(trailer_heading);
    
    
    fprintf('num_i = %f \n',num_i); 
    fprintf('truck_linear_v = %f \n',truck_linear_v); 
    tic
    if(num_i == 1)
        truck_path_points = readPlanPoints();
        for i = 1:1:size(truck_path_points.x )
            truck_path_points.x(i) =  truck_path_points.x(i) - 4.25;
%             truck_path_points.v(i) =  truck_path_points.v(i) - 10;
            truck_path_points.v(i) =  15;
        end
        target_points = truck_path_points;
        % generate the trajectory for the trailer from the truck trajectory
        % using a kinematic truck model
        trailer_path_points = generateTrailerPoints(truck_path_points, "kinematic",CONST_hF,CONST_hR);
        trailer_points = trailer_path_points;       
    end
    
    matrix_A =zeros(basic_state_size,basic_state_size);
    matrix_Ad =zeros(basic_state_size,basic_state_size); %vehicle state matrix (discrete-time)
    matrix_B =zeros(basic_state_size,1);
    matrix_Bd =zeros(basic_state_size,1);                       % control matrix (discrete-time)
    matrix_D1d =zeros(basic_state_size,1);                       % control matrix (discrete-time)
    matrix_D2d =zeros(basic_state_size,1);                       % control matrix (discrete-time)
    matrix_K = zeros(1,basic_state_size);
    
    matrix_Q = zeros(basic_state_size,basic_state_size);
    matrix_R = eye(1,1);
    matrix_state = zeros(basic_state_size,1);
            
    if(truck_linear_v < 0.2)   %
        Vx = 0.2;
    else
        Vx = truck_linear_v;
    end

    truckmodel.set_lon_speed(Vx);
    
    

    matrix_Q(1,1) = 25.02; %  0.01145 0.58245
    matrix_Q(2,2) = 0.00;
    matrix_Q(3,3) = 100.0000; %  0.0120  4.0000
    matrix_Q(4,4) = 0.0000;
    
    matrix_Q(5,5) = 5.2; %  0.01145 0.58245
    matrix_Q(6,6) = 0.00;
    matrix_Q(7,7) = 100.000; %  0.0120  4.0000
    matrix_Q(8,8) = 0.0000;
    
    %% matrix_state init
%      [com_x,com_y] =ComputeCOMPosition(3.25,0,truck_x,truck_y,0);%compute COG based on truck position
[truck_lateral_err,truck_lateral_err_rate,truck_heading_err,truck_heading_err_rate, ...
    trailer_lateral_err, trailer_lateral_err_rate, ...
    trailer_heading_err, trailer_heading_err_rate, index_min] = ...
    ComputeLateralErrors(truck_x,truck_y,trailer_x,trailer_y, ...
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
    
% Consider this code instead:
%     heading_err = wrapToPi(heading_err);
%     trail_heading_err = wrapToPi(trail_heading_err);
    h = max(150, length(target_points.x) - index_min);
%     arr_truck_angular_v = ...
%        target_points.curvature(index_min:index_min+h-1).*target_points.v(index_min:index_min+h-1);
%     arr_trailer_angular_v = ...
%        trailer_points.curvature(index_min:index_min+h-1).*trailer_points.v(index_min:index_min+h-1);
    truck_angular_v = ...
       target_points.curvature(index_min).*target_points.v(index_min);
    trailer_angular_v = ...
       trailer_points.curvature(index_min).*trailer_points.v(index_min);
    
    matrix_state(1,1) = truck_lateral_err;
    matrix_state(2,1) = truck_lateral_err_rate;
    matrix_state(3,1) = truck_heading_err;
    matrix_state(4,1) = truck_heading_err_rate;
    matrix_state(5,1) = trailer_lateral_err;
    matrix_state(6,1) = trailer_lateral_err_rate;
    matrix_state(7,1) = trailer_heading_err;
    matrix_state(8,1) = trailer_heading_err_rate;
    fprintf('truck_lateral_err = %f \n' , truck_lateral_err);
        
    %% discretize matrix
    matrix_I = eye(basic_state_size,basic_state_size);
    matrix_Ad = truckmodel.Ad(t_s);    
    matrix_Bd = truckmodel.Cd(t_s);
%     matrix_D1d = truckmodel.D1d(t_s,1);
%     matrix_D2d = truckmodel.D2d(t_s,2);
    matrix_D1d = truckmodel.D1d(t_s,truck_heading_err_rate);
    matrix_D2d = truckmodel.D2d(t_s,trailer_heading_err_rate);
    
    %% solve LQR problem

    [truck_ratio_lat,truck_ratio_heading, ...
        trailer_ratio_lat,trailer_ratio_heading] = Interpolate8(truck_linear_v);
    matrix_Q(1,1) = truck_ratio_lat *  matrix_Q(1,1);
    matrix_Q(3,3) = truck_ratio_heading *  matrix_Q(3,3);
    matrix_Q(5,5) = trailer_ratio_lat *  matrix_Q(5,5);
    matrix_Q(7,7) = trailer_ratio_heading *  matrix_Q(7,7);
    h = 10;
    max_steering_angle = deg2rad(steer_single_direction_max_degree_);
    control = SolveCiDiMPCProblem(matrix_Ad, matrix_Bd, matrix_D1d, ...
      matrix_D2d, matrix_Q, matrix_R, h, matrix_state,truck_angular_v,trailer_angular_v, ...
      max_steering_angle);

    steer_angle_feedback = (control(1,1) ) * 180 / pi * steer_transmission_ratio_;
%     matrix_K = SolveLQRProblem(matrix_Ad,matrix_Bd,matrix_Q,matrix_R,lqr_eps_,lqr_max_iteration_);
%     steer_angle_feedback = -(matrix_K * matrix_state) * 180 / pi * steer_transmission_ratio_;
    
    %%   compute feedforward angle
%     kv =  lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
%     steer_angle_feedforwardterm =(wheelbase_ * target_points.curvature(index_min) + ...
%                                                       kv * truck_linear_v * truck_linear_v * target_points.curvature(index_min) - ...
%                                                        matrix_K(1, 3) *(lr_ * target_points.curvature(index_min) - ...
%                                                        lf_ * mass_ * truck_linear_v * truck_linear_v * target_points.curvature(index_min) ...
%                                                        / 2 / cr_ / wheelbase_)) *180 / pi * steer_transmission_ratio_ ;
    steer_angle_feedforwardterm = (CONST_hR + CONST_hF) * target_points.curvature(index_min) *180 / pi * steer_transmission_ratio_;
%     steer_angle_feedforwardterm = 0;
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
        truck_heading_err, ...
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
%     output(2,1) = 70;  %unit km/h
    output(2,1) = target_points.v(index_min)  * 3.6;  %unit km/h
    
    fprintf('index_min = %f \n',index_min);
    t_sim = 45 ;%51.5  30
    distance_to_end_point = sqrt( (truck_x - target_points.x(size(target_points.x , 1)))^2 + (truck_y - target_points.y(size(target_points.y , 1))) ^2 );
    if (num_i == (t_sim * 100+1) || distance_to_end_point < 0.2 )
%     if (num_i == (t_sim * 100+1) )       
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
        writetable(T,'E:\CIDI_SOFT\Program_File\Trucksim2016.1\TruckSim2016.1_Data\truckStateFollowing_.csv');
        
    end
        
    previousOutput(1,1) = output(1,1);
    previousOutput(2,1) = output(2,1);  
    sys = output;
   
    toc
    





