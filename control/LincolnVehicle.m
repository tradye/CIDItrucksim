classdef LincolnVehicle < handle
    properties
        time = 0
        truck_X = 0
        truck_Y = 0
        truck_heading = 0
        truck_heading_rate = 0
        truck_vx = 0
        err_hist = [0 0 0 0]
        path_hist = [0 0 0 0]
        control_hist = [];
        truck_model
        truck_path
        e1 = 0
        de1 = 0
        theta1 = 0
        dtheta1 = 0
    end % properties
    methods
        function obj = LincolnVehicle(truck_model, truck_path)
            obj.truck_model= LincolnModel();            
            obj.truck_model = copy(truck_model); % make sure this is a deep copy
            obj.truck_path = truck_path;
            obj.err_hist = [obj.e1 obj.de1 obj.theta1 obj.dtheta1];
        end % LincolnVehicle constructor      
        function val = get_truck_states(obj)
            val = [obj.truck_X; ...
                   obj.truck_Y; ...
                   obj.truck_heading; ...
                   obj.truck_heading_rate; ...
                   obj.truck_vx];
        end % get_truck_states        
        function value = get_state_error(obj)
            % Return tracking error state space vector based on
            % current vehicle position
            xyd = obj.truck_path.xy;
            truckp = obj.truck_path;
            % Get index of closest point on refLine to position
            [~, ind] = min((obj.truck_X - xyd(:,1)).^2 + ...
                (obj.truck_Y - xyd(:,2)).^2);
            
            % matched truck path
            truck_X_d = truckp.data.x(ind);
            truck_Y_d = truckp.data.y(ind);
            truck_heading_d = truckp.angle(ind);
            truck_heading_rate_d = truckp.angle_rate(ind);
            
            % Calculate tracking error variables
            dx = obj.truck_X - truck_X_d;
            dy = obj.truck_Y - truck_Y_d;            
            truck_lateral_error = cos(obj.truck_heading)*dx - ... 
                sin(obj.truck_heading)*dy;            
            truck_heading_error = obj.truck_heading - truck_heading_d;
            truck_vy_error = obj.truck_vx*sin(truck_heading_error);
            truck_heading_error_rate = obj.truck_heading_rate - ...
                truck_heading_rate_d;                        
            truck_errs = [truck_lateral_error; truck_vy_error; ...
                truck_heading_error; truck_heading_error_rate];
            value = [truck_errs];
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
            
            truckp = obj.truck_path;            
            ind = obj.truck_path.get_path_index_by_time(obj.time + dt);
            
            truck_X_d = truckp.data.x(ind);
            truck_Y_d = truckp.data.y(ind);
            truck_heading_d = truckp.angle(ind);
            truck_heading_rate_d = truckp.angle_rate(ind);
            truck_speed_d = truckp.speedProfile(ind);
            
            I = eye(4);
            Ad = (I + 0.5*inv(M)*A*dt)*inv(I -0.5*inv(M)*A*dt);
            Cd = inv(M)*C*dt;
            D1d = inv(M)*D1*dt;
            
            feed_forward = ...
                (obj.truck_model.b1+obj.truck_model.b2)*truck_heading_d/truck_speed_d;
            actual_control = front_axle_angle + feed_forward;
            old_errs = [obj.e1; obj.de1; obj.theta1; obj.dtheta1];            
             new_errs = Ad*old_errs + Cd*(actual_control) + ...
                 D1d*truck_heading_rate_d;
             
            obj.e1 = new_errs(1);
            obj.de1 = new_errs(2);
            obj.theta1 = new_errs(3);
            obj.dtheta1 = new_errs(4);            

            obj.truck_X = truck_X_d - obj.e1 * ...
                sin(obj.theta1 + truck_heading_d);
            obj.truck_Y = truck_Y_d + obj.e1 * ...
                cos(obj.theta1 + truck_heading_d);
            obj.truck_heading = obj.theta1 + truck_heading_d;
            obj.truck_heading_rate = obj.dtheta1 + ...
                truck_heading_rate_d;
            obj.truck_vx = truck_speed_d; % TODO: come from controller
            
            obj.control_hist = [obj.control_hist; actual_control];
            obj.err_hist = [obj.err_hist; transpose(new_errs)];
            obj.path_hist = [obj.path_hist; obj.truck_X, obj.truck_Y, ...
                obj.truck_heading, obj.truck_heading_rate];

            obj.time = obj.time + dt;
            val = actual_control;
        end % move
        function run(obj, controller, ts)
            for i = 1: floor(obj.truck_path.cum_time_elapsed(end))/ts
                t = i*ts;
                control = 0;
                if strcmp(class(controller),'LQRController')
                    control = -controller.K(obj.truck_vx, ts)*obj.get_state_error();
                end
                act_control = obj.move(control, ts);
            end
        end % run
        function plot(obj)
            hold on
            subplot(3,2,1);
            plot(obj.truck_path.data.x, obj.truck_path.data.y, 'r',obj.path_hist(:,1), obj.path_hist(:,2),'g')
            xlabel('X (m)');
            ylabel('Y (m)');            
            subplot(3,2,2);
            plot(obj.truck_path.cum_time_elapsed, obj.truck_path.angle, 'r');
            xlabel('Time (s)');
            ylabel('Heading (rad)');
            subplot(3,2,3);
            plot(obj.truck_path.cum_time_elapsed, obj.truck_path.angle_rate, 'r');
            xlabel('Time (s)')
            ylabel('Heading Rate (rad/s)');
            subplot(3,2,4);
            x = linspace(1,length(obj.control_hist),length(obj.control_hist));
            ts = obj.time/length(x);
            x = x*ts;
            plot(x, obj.control_hist);
            xlabel('Time (s)');
            ylabel('Control Angle');
            subplot(3,2,5);
            plot(x,obj.err_hist(2:end,1),'r');
            xlabel('Time (s)');            
            ylabel('Lateral Error (m)');
            subplot(3,2,6);
            plot(x,obj.err_hist(2:end,3),'r');
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
    end % methods
end % classdef