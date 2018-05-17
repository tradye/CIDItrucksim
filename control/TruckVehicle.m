classdef TruckVehicle < handle
    properties
        time = 0
        truck_X = 0
        truck_Y = 0
        truck_heading = 0
        truck_heading_rate =0
        truck_vx = 0
        trailer_X = 0
        trailer_Y = 0
        trailer_heading = 0
        trailer_heading_rate = 0        
        truck_model = 0
        truck_path = 0
        trailer_path % generated trailer path from truck model
        e1 = 0
        de1 = 0
        theta1 = 0
        dtheta1 = 0
        e2 = 0
        de2 = 0
        theta2 = 0
        dtheta2 = 0
        err_hist = [0 0 0 0 0 0 0 0]
        path_hist = [0 0 0 0 0 0 0 0]
        pre_control_status = []
        control_hist = []
        max_steering_angle = 0.5        
    end % properties
    methods
        function obj = TruckVehicle(truck_model, truck_path, trailer_method, trailer_path)
            obj.truck_model= TruckModel6Axle();            
            obj.truck_model = copy(truck_model); % make sure this is a deep copy
            obj.truck_path = truck_path;
            if nargin == 4 && trailer_method == "explicit"
               obj.trailer_path.x = trailer_path.data.x;
               obj.trailer_path.y = trailer_path.data.y;
               obj.trailer_path.angle = trailer_path.angle;              
               obj.trailer_path.angle_rate = trailer_path.angle_rate;
            elseif nargin == 3
                obj.get_trailer_path(trailer_method);
            elseif nargin == 2
                obj.get_trailer_path("kinematic")
            end
                obj.err_hist = [obj.e1, obj.de1, obj.theta1, obj.dtheta1, ...
                obj.e2, obj.de2, obj.theta2, obj.dtheta2];
        end % TruckVehicle constructor
        function obj = get_trailer_path(obj,method)
            if method == "dup" % duplicate the truck path
               obj.trailer_path.x = obj.truck_path.data.x;
               obj.trailer_path.y = obj.truck_path.data.y;
               obj.trailer_path.angle = obj.truck_path.angle;              
               obj.trailer_path.angle_rate = obj.truck_path.angle_rate;            
            elseif method == "kinematic"
                tp = obj.truck_path;
                obj.trailer_path.x = [-obj.truck_model.hF - obj.truck_model.hR];
                obj.trailer_path.y = [0];
                obj.trailer_path.angle = [0];
                obj.trailer_path.angle_rate = [0];
                for i=2:length(tp.x)
                    x0 = tp.x(i);
                    y0 = tp.y(i);    
                    v0 = tp.speedProfile(i);    
                    obj.trailer_path.angle(i) = wrapToPi(obj.trailer_path.angle(i-1) + ...
                        v0/obj.truck_model.hR* ...
                        sin(obj.truck_path.angle(i-1) - obj.trailer_path.angle(i-1))* ...
                        tp.time_elapsed(i));
                    %obj.trailer_path.angle_rate(i) = ... 
                    %    (obj.trailer_path.angle(i) - obj.trailer_path.angle(i-1)) / ...
                    %    tp.time_elapsed(i);                                    
                    obj.trailer_path.x(i) = obj.truck_path.x(i) - ...
                        obj.truck_model.hF*cos(obj.truck_path.angle(i)) - ...
                        obj.truck_model.hR*cos(obj.trailer_path.angle(i));
                    obj.trailer_path.y(i) = obj.truck_path.y(i) - ... 
                        obj.truck_model.hF*sin(obj.truck_path.angle(i)) - ...
                        obj.truck_model.hR*sin(obj.trailer_path.angle(i));                  
                end % for i
                obj.trailer_path.angle_rate = ...
                        tp.get_angle_rates(obj.trailer_path.angle, tp.time_elapsed);
            elseif method == "dynamic"
                tp = obj.truck_path;
                obj.trailer_path.x = [-obj.truck_model.hF - obj.truck_model.hR];
                obj.trailer_path.y = [0];
                Ky = [0];
                Ly = [0];
                obj.trailer_path.angle = [0];
                obj.trailer_path.angle_rate = [0];
                for i=2:length(tp.x)             
                    if i == 2
                        % need to infer i-2 number as it is not calculated   
                        k_lateral_rate0 = 0;
                        l_lateral_rate0 = 0;
                    else                        
                        k_lateral_rate0 = (tp.y(i-1) - tp.y(i-2)) / ...
                            tp.time_elapsed(i-1) * cos(tp.angle(i-1));    
                        l_lateral_rate0 = (obj.trailer_path.y(i-1) - ...
                        tp.y(i-2)) / ...                        
                        tp.time_elapsed(i-1) * cos(obj.trailer_path.angle(i-1));
                        k_lateral_rate0 = Ky(i-1);
                        l_lateral_rate0 = Ly(i-1);
                    end % if i == 2
                    C4 = obj.truck_model.C4;
                    C5 = obj.truck_model.C5;
                    C6 = obj.truck_model.C6;
                    hR = obj.truck_model.hR;
                    hF = obj.truck_model.hF;                    
                    b4 = obj.truck_model.b4;
                    b5 = obj.truck_model.b5;
                    b6 = obj.truck_model.b6;
                    m2 = obj.truck_model.m2;
                    I2 = obj.truck_model.I2;
                    v1 = tp.speedProfile(i);
                    
                    Cy = (hR*(C4+C5+C6) + b4*C4+b5*C5+b6*C6)/(v1);
                    Cp = (-hR*(b4*C4+b5*C5+b6*C6) - ...
                        (C4*b4^2 + C5*b5^2 + C6*b6^2))/v1 + hR*m2*v1;
                                        
                    dt = tp.time_elapsed(i);
                    
                    k_heading_rate0 = tp.angle_rate(i-1);
                    k_heading_rate1 = tp.angle_rate(i);
                    k_heading = tp.angle(i);
                    %k_lateral_rate1 = (tp.y(i) - tp.y(i-1)) / ...
                    %    tp.time_elapsed(i);                    
                    
                    k_lateral_rate1 = (tp.y(i) - tp.y(i-1))/dt * cos(k_heading);
                    Ky(i) = k_lateral_rate1;  
                    
                    l_heading_rate0 = obj.trailer_path.angle_rate(i-1);                                                                               
                    Formula1 = Cy*l_lateral_rate0 + Cp*l_heading_rate0;
                    Formula2 = -v1*k_heading_rate0 + v1*l_heading_rate0 + ...
                        (k_lateral_rate1 - k_lateral_rate0)/dt + ...
                        hF*(k_heading_rate1 - k_heading_rate0)/dt;
                    l_acc = (Formula1 - hR*m2*Formula2)/(m2*hR^2 + I2);
                    l_heading_rate1 = l_heading_rate0 + l_acc*dt;
                    
                    l_heading = obj.trailer_path.angle(i-1) + l_heading_rate1*dt;
                    
                    % calculate trailer position
                    obj.trailer_path.x(i) = tp.x(i) - ...
                        hF*cos(k_heading) - ...
                        hR*cos(l_heading);
                    obj.trailer_path.y(i) = tp.y(i) - ... 
                        hF*sin(k_heading) - ...
                        hR*sin(l_heading);  
                                        
                    Ly(i) = (obj.trailer_path.y(i) - obj.trailer_path.y(i-1)) ...
                        /dt * cos(l_heading);
                    obj.trailer_path.angle(i) = l_heading;
%                     obj.trailer_path.angle_rate(i) = ...
%                         (obj.trailer_path.angle(i) - ...
%                         obj.trailer_path.angle(i-1)) / ...
%                         dt;
                    obj.trailer_path.angle_rate(i) = l_heading_rate1;                                    
                end % if i=2:length(tp,x)
%                 obj.trailer_path.angle_rate = ...
%                         tp.get_angle_rates(obj.trailer_path.angle, tp.time_elapsed);                
            elseif method == "dynamicsolve"
                CONST_TOL = 0.01;
                CONST_MAXITER = 150;
                controller = LQRController(obj.truck_model, CONST_TOL, CONST_MAXITER);
                Q = blkdiag(1,  0, 1, 0, 0, 0, 0, 0);
                R = 1;
                % Q = blkdiag(0,  0.1, 0.5, 0, 0, 0.1, 0.5, 0);
                % R = 0.0015;
                controller.SetQR(Q, R);
                gain_speeds = [4.0, 8.0, 12.0, 20.0, 25.0];
                lateral_gain = [0.6, 0.4, 0.2, 0.1, 0.05];
                heading_gain = [1.0, 0.6, 0.1, 0.1, 0.1];
                controller.SetGainSchedule(gain_speeds, ...
                    [lateral_gain; ones(1,length(gain_speeds)); ...
                    heading_gain; ones(1,length(gain_speeds)); ...
                    lateral_gain; ones(1, length(gain_speeds)); ...
                    heading_gain; ones(1, length(gain_speeds)) ...
                    ]);                                        
                trailer_solver = TruckVehicle(obj.truck_model, obj.truck_path, "dup");
                ts = 0.02;
                trailer_solver.run(controller, ts);                    
                truck_track = trailer_solver.get_truck_cidipath();
                trailer_track = trailer_solver.get_trailer_cidipath();

                obj.truck_path = truck_track;

                obj.trailer_path.x = trailer_track.x;
                obj.trailer_path.y = trailer_track.y;
                obj.trailer_path.angle = trailer_track.angle;              
                obj.trailer_path.angle_rate = trailer_track.angle_rate;   
            elseif method == "dynamicregression"                
                % Generate a truck model and truck vehicle
                % move the vehicle by 
            end % if method == ""
        end % get_trailer_path        
        function val = get_truck_states(obj)
            val = [obj.truck_X; ...
                   obj.truck_Y; ...
                   obj.truck_heading; ...
                   obj.truck_heading_rate; ...
                   obj.truck_vx];
        end % get_truck_states
        function val = get_trailer_states(obj)
            val = [obj.trailer_X; ...
                   obj.trailer_Y; ...
                   obj.trailer_heading; ...
                   obj.trailer_heading_rate; ...
                   obj.truck_vx];
        end % get_trailer_states
        function val = get_trailer_xy(obj, truck_x, truck_y, ...
                truck_heading, trailer_heading)
            % Returns [trailer_x, trailer_y]
            hF = obj.truck_model.hF;
            hR = obj.truck_model.hR;
            trailer_dx = hF*cos(truck_heading) + hR*cos(trailer_heading);
            trailer_dy = hF*sin(truck_heading) + hR*sin(trailer_heading);
            val = [truck_x - trailer_dx, truck_y - trailer_dy];
        end
        function value = get_state_error(obj)
            % Return tracking error state space vector based on
            % current vehicle position
            xyd = obj.truck_path.xy;
            truckp = obj.truck_path;
            trailp = obj.trailer_path;
            % Get index of closest point on refLine to position
            [~, ind] = min((obj.truck_X - xyd(:,1)).^2 + ...
                (obj.truck_Y - xyd(:,2)).^2);
            
            % matched truck path
            truck_X_d = truckp.data.x(ind);
            truck_Y_d = truckp.data.y(ind);
            truck_heading_d = truckp.angle(ind);
            truck_heading_rate_d = truckp.angle_rate(ind);
            
            % matched trailer path
            trailer_X_d = trailp.x(ind);
            trailer_Y_d = trailp.y(ind);
            trailer_heading_d = trailp.angle(ind);
            trailer_heading_rate_d = trailp.angle_rate(ind);                        
            
            % Calculate tracking error variables
            dx = obj.truck_X - truck_X_d;
            dy = obj.truck_Y - truck_Y_d;            
            truck_lateral_error = cos(obj.truck_heading)*dy - ... 
                sin(obj.truck_heading)*dx;            
            truck_heading_error = obj.truck_heading - truck_heading_d;
            truck_vy_error = obj.truck_model.Vx*sin(truck_heading_error);
            truck_heading_error_rate = obj.truck_heading_rate - ...
                truck_heading_rate_d;                        
            truck_errs = [truck_lateral_error; truck_vy_error; ...
                truck_heading_error; truck_heading_error_rate];
            
            dx = obj.trailer_X - trailer_X_d;
            dy = obj.trailer_Y - trailer_Y_d;            
            trailer_lateral_error = cos(obj.trailer_heading)*dy - ... 
                sin(obj.trailer_heading)*dx;            
            trailer_heading_error = obj.trailer_heading - trailer_heading_d;
            trailer_vy_error = obj.truck_model.Vx*sin(trailer_heading_error);
            trailer_heading_error_rate = obj.trailer_heading_rate - ...
                trailer_heading_rate_d;                        
            trailer_errs = [trailer_lateral_error; trailer_vy_error; ...
                trailer_heading_error; trailer_heading_error_rate];
            
            value = [truck_errs; trailer_errs];
        end % get_state_vector
        function val = move(obj, front_axle_angle, dt)            
            % Move
            % Take e0 -> SYS + front_wheel_angle -> e1
            % Need to get next period x, y, heading from e1 and path
            % e
            M = obj.truck_model.M();
            A = obj.truck_model.A();
            C = obj.truck_model.C();
            D1 = obj.truck_model.D1();
            D2 = obj.truck_model.D2();            
            
            truckp = obj.truck_path;
            trailp = obj.trailer_path;
            ind = obj.truck_path.get_path_index_by_time(obj.time + dt);
            
            truck_X_d = truckp.data.x(ind);
            truck_Y_d = truckp.data.y(ind);
            truck_heading_d = truckp.angle(ind);
            truck_heading_rate_d = truckp.angle_rate(ind);
            %truck_speed_d = truckp.speedProfile(ind);
            truck_speed_d = truckp.speedProfile(ind);
            
            trailer_X_d = trailp.x(ind);
            trailer_Y_d = trailp.y(ind);
            trailer_heading_d = trailp.angle(ind);
            trailer_heading_rate_d = trailp.angle_rate(ind);
            
            I = eye(8);
            Ad = (I + 0.5*inv(M)*A*dt)*inv(I -0.5*inv(M)*A*dt);
            Cd = inv(M)*C*dt;
            D1d = inv(M)*D1*dt;
            D2d = inv(M)*D2*dt;

            feed_forward = ...
                ((obj.truck_model.b1 + obj.truck_model.b3)* ...
                (truck_heading_rate_d/obj.truck_model.Vx)-1* ...
                (obj.truck_model.hR + obj.truck_model.b6)* ...
                (trailer_heading_rate_d/obj.truck_model.Vx));
           
            actual_control = front_axle_angle + feed_forward;
            old_errs = [obj.e1; obj.de1; obj.theta1; obj.dtheta1; ...
                obj.e2; obj.de2; obj.theta2; obj.dtheta2];            
             new_errs = Ad*old_errs + Cd*(actual_control) + ...
                 D1d*truck_heading_rate_d + D2d*trailer_heading_rate_d;

            obj.e1 = new_errs(1);
            obj.de1 = new_errs(2);
            obj.theta1 = new_errs(3);
            obj.dtheta1 = new_errs(4);
            obj.e2 = new_errs(5);
            obj.de2 = new_errs(6);
            obj.theta2 = new_errs(7);
            obj.dtheta2 = new_errs(8);
            
            obj.truck_X = truck_X_d - obj.e1 * ...
                sin(obj.theta1 + truck_heading_d);
            obj.truck_Y = truck_Y_d + obj.e1 * ...
                cos(obj.theta1 + truck_heading_d);
            obj.truck_heading = obj.theta1 + truck_heading_d;
            obj.truck_heading_rate = obj.dtheta1 + ...
                truck_heading_rate_d;
            obj.truck_vx = obj.truck_model.Vx; % TODO: come from controller
            
            obj.trailer_X = trailer_X_d - obj.e2 * ...
                sin(obj.theta2 + trailer_heading_d);
            obj.trailer_Y = trailer_Y_d + obj.e2 * ...
                cos(obj.theta2 + trailer_heading_d);
            obj.trailer_heading = obj.theta2 + trailer_heading_d;
            obj.trailer_heading_rate = obj.dtheta2 + ...
                trailer_heading_rate_d;
            get_trailer_xy = obj.get_trailer_xy(obj.truck_X, obj.truck_Y, ...
                obj.truck_heading, obj.trailer_heading);
            obj.trailer_X = get_trailer_xy(1);
            obj.trailer_Y = get_trailer_xy(2);
            
            obj.control_hist = [obj.control_hist; actual_control];
            obj.path_hist = [obj.path_hist; obj.truck_X, obj.truck_Y, ...
                obj.truck_heading, obj.truck_heading_rate, ...
                obj.trailer_X, obj.trailer_Y, obj.trailer_heading, ...
                obj.trailer_heading_rate];

            redo_errs = obj.get_state_error();
            obj.e1 = redo_errs(1);          
            obj.theta1 = redo_errs(3);          
            obj.e2 = redo_errs(5);      
            obj.theta2 = redo_errs(7);
%             obj.err_hist = [obj.err_hist; redo_errs'];
            obj.err_hist = [obj.err_hist; ...
               obj.e1 obj.de1 obj.theta1 obj.dtheta1 ...
               obj.e2 obj.de2 obj.theta2 obj.dtheta2];

            
            obj.time = obj.time + dt;
            val = actual_control;
        end % move
        function run(obj, controller, ts)
            tp = obj.truck_path;
            for i = 1: floor(obj.truck_path.cum_time_elapsed(end))/ts               
                t = i*ts;
                ind = obj.truck_path.get_path_index_by_time(t);
                obj.pre_control_status = [obj.pre_control_status; ...
                    tp.x(ind) tp.y(ind) tp.angle(ind) tp.angle_rate(ind)];
                control = 0;
                if strcmp(class(controller),'LQRController')
                    control = -controller.K(obj.truck_vx, ts)*obj.get_state_error();
                end
                if strcmp(class(controller),'MPCController')
                   CONST_HORIZON = 20;
                   control = controller.control(obj.truck_vx, CONST_HORIZON, ...
                       ts, obj.err_hist(i,:), ...
                       obj.truck_path.angle_rate(ind), ...
                       obj.trailer_path.angle_rate(ind), ...
                       obj.max_steering_angle);
                   control = control(1);
                end
                act_control = obj.move(control, ts);                
            end
        end % run
        function plot(obj)
                hold on
                subplot(3,2,1);
                plot(obj.truck_path.x, obj.truck_path.y, 'r', ...
                    obj.path_hist(:,1), obj.path_hist(:,2),'g', ...
                    obj.trailer_path.x, obj.trailer_path.y, 'b', ...
                    obj.path_hist(:,5), obj.path_hist(:,6), 'k')
                xlabel('X (m)');
                ylabel('Y (m)');            
                subplot(3,2,2);
                plot(obj.truck_path.cum_time_elapsed, ...
                    obj.truck_path.angle, 'r', ...
                    obj.truck_path.cum_time_elapsed, ...
                    obj.trailer_path.angle, 'b');
                xlabel('Time (s)');
                ylabel('Heading (rad)');
                subplot(3,2,3);
                plot(obj.truck_path.cum_time_elapsed, ...
                    obj.truck_path.angle_rate, 'r', ...
                    obj.truck_path.cum_time_elapsed, ...
                    obj.trailer_path.angle_rate, 'b');
                xlabel('Time (s)')
                ylabel('Heading Rate (rad/s)');
                subplot(3,2,4);
                x = linspace(1,length(obj.control_hist), ...
                    length(obj.control_hist));
                ts = obj.time/length(x);
                x = x*ts;
                plot(x, obj.control_hist);
                xlabel('Time (s)');
                ylabel('Control Angle');
                subplot(3,2,5);
                plot(x,obj.err_hist(2:end,1),'r', ...
                    x,obj.err_hist(2:end,5), 'b');
                xlabel('Time (s)');            
                ylabel('Lateral Error (m)');
                subplot(3,2,6);
                plot(x,obj.err_hist(2:end,3),'r', ...
                    x,obj.err_hist(2:end,6),'b');
                xlabel('Time (s)');            
                ylabel('Heading Error (rad)');
                hold off
            end % plot
            function path = write_path(obj, xys, start_speed, end_speed, max_speed)
                poses = [];                
                for i=1:size(xys,1)
                    if mod(i,10) == 0
                        poses = [poses; xys(i, 1:3)];
                    end
                end
                CONST_ANGLE_ROUND = 15;
                CONST_NUM_POINTS = 10;
                poses(:,3) = round(rad2deg(wrapToPi(poses(:,3))/ ...
                    CONST_ANGLE_ROUND))*CONST_ANGLE_ROUND;
                pathCustom = cidiPath(poses, start_speed, ...
                    end_speed, max_speed, CONST_NUM_POINTS);
                path = pathCustom;
            end % write_path                           
            function val = get_truck_cidipath(obj)
                val = obj.write_path(obj.path_hist(:,1:3), ...
                    obj.truck_vx, obj.truck_vx, obj.truck_vx);
            end % get_truck_cidipath
            function val = get_trailer_cidipath(obj)
                val = obj.write_path(obj.path_hist(:,5:7), ...
                    obj.truck_vx, obj.truck_vx, obj.truck_vx);
            end % get_truck_cidipath            
    end % methods
end % classdef