classdef cidiPath < handle
    % A class that encapsulates MATLAB ADAS Path and Spline creation
    % Note that the vectors in this object are indexed by SPLINE index,
    % not by time or any measure of distance
    % Methods will be added later that allow calculation by distance.
    properties (Access = private)
        drivingPathObject %  the MATLAB built-in Path Object        
    end % properties (Access = private)
    properties (Dependent)
        Poses % the matrix of Poses
    end % properties (Dependent)
    properties
        numPoints % number of estimated points on the Spline
        speedProfile % speed at each point on the spline
        data % data object representing a MATLAB path spline
        x % array of x points
        y % array of y points
        angle % angle at spline index
        angle_rate % angle rate at spline index
        distance_elapsed % elapsed distance at spline index   
        time_elapsed % elapsed time at spline index
        cum_time_elapsed % cumulative time elapsed at spline index
        xy % a matrix of (x,y) positions
    end % Properties    
    methods
        function obj = cidiPath(poses, speedStart, speedEnd, speedMax, deltaPoints)
            % Creator Method
            % - Poses is a (N x 3) matrix. Each row is:
            %   (global_x_position, global_y_position, heading_angle)
            %   The first pose should be (0, 0, 0).
            % - speedStart, speedEnd, speedMax are speed markers for the path in m/s.
            %   Use the same number for all three, for now.
            % - deltaPoints is the number of estimated points on the Spline between
            %   each point. Not that important for now. Use 250
            obj.numPoints = size(poses, 1)*deltaPoints;
            obj.drivingPathObject = driving.Path.create(poses, 'Reed-Shepp');
            splineFitter = HelperCubicSplineFit(obj.drivingPathObject);
            obj.data = fit(splineFitter, obj.numPoints);     
            
            speedProfileGenerator = HelperSpeedProfileGenerator(obj.data);
            speedProfileGenerator.StartSpeed = speedStart;
            speedProfileGenerator.EndSpeed   = speedEnd;
            speedProfileGenerator.MaxSpeed   = speedMax;
            obj.speedProfile = generate(speedProfileGenerator);                        
            
            obj.angle = transpose(arrayfun(@(x) atan2(obj.data.dy(x),obj.data.dx(x)), ...
                linspace(1, obj.numPoints, obj.numPoints)));
            %obj.angle_rate = transpose((obj.speedProfile.*obj.data.kappa));            
            obj.xy = [obj.data.x, obj.data.y];
            obj.distance_elapsed = [0; sqrt((obj.data.dx(2:end).*diff(obj.data.ts)).^2+(obj.data.dy(2:end).*diff(obj.data.ts)).^2)];
            obj.time_elapsed = obj.distance_elapsed./obj.speedProfile;
            %obj.angle_rate = [0; diff(obj.angle)./obj.time_elapsed(2:end)];
            obj.angle_rate = obj.get_angle_rates(obj.angle, obj.time_elapsed);
            obj.cum_time_elapsed = cumsum(obj.time_elapsed);
        end % cidiPath Constructor
        function val = get_angle_rates(obj, arrAngles, arrTimes)
            result = [0];
            for i=2:length(arrAngles)
                angle1 = arrAngles(i-1);
                angle2 = arrAngles(i);
                if angle1 <= pi && angle1 > pi/2 && angle2 > -pi && angle2 < -pi/2
                    diff = (angle2 + 2*pi) - angle1;
                elseif angle1 >= -pi && angle1 < -pi/2 && angle2 < pi && angle2 > pi/2
                    diff = angle2 - (angle1 + 2*pi);
                else
                    diff = angle2 - angle1;
                end
                result = [result; diff/arrTimes(i)];                
            end % i=1:length(arrAngles)
            val = result;            
        end % get_angle_rates
        function obj = plot(obj)
            % Plot global X,Y position, heading, heading rate, and speed
            % profile
            hold on
            subplot(2,2,1);
            x_waypoints = transpose(obj.drivingPathObject.KeyPoses(1:end,1));
            y_waypoints = transpose(obj.drivingPathObject.KeyPoses(1:end,2));
            hSmoothPath = plot(obj.data.x, obj.data.y, 'r', x_waypoints,y_waypoints,'o', 'LineWidth', 2, ...
                'DisplayName', 'Smoothed Path');
            xlabel('Global X Position (m)');
            ylabel('Global Y Position (m)');
            subplot(2,2,2);
            hSmoothTheta = plot(obj.data.ts, rad2deg(obj.angle), 'b','LineWidth', 2);
            xlabel('Waypoint Reference Index');
            ylabel('\theta (Deg)');
            subplot(2,2,3);
            hSmoothThetaDot = plot(obj.data.ts, rad2deg(obj.angle_rate), 'g','LineWidth', 2);
            ylabel('\theta Rate (Deg/sec)');
            xlabel('Waypoint Reference Index');
            subplot(2,2,4);
            hSpeed = plot(obj.data.ts, obj.speedProfile, 'v', 'LineWidth', 1);
            ylabel('Linear Velocity (m/s)');
            xlabel('Waypoint Reference Index');
            hold off
        end % plot
        function value = get.Poses(obj)
            % Pose Matrix helper method
            value = obj.drivingPathObject.KeyPoses;
        end % get.Poses
        function value = get.x(obj)
            value = obj.data.x;
        end % get.x
        function value = get.y(obj)
            value = obj.data.y;
        end % get.x
        function val = get_path_index_by_time(obj, t)
            % Given time t, return index of data of closest time
            % Should time t be PAST t?
            [~, val] = min((t - obj.cum_time_elapsed).^2);
        end % get_path_index
    end % methods
end  % classdef
