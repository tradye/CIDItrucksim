classdef MPCController < handle
    properties
        model
        Q
        R
        array_speed = [4.0, 8.0, 12.0, 20.0, 25.0];
        mat_ratio
    end % properties
    methods
        function obj = MPCController(model)
            obj.model = model;
            obj.mat_ratio = ones(model.numStates, length(obj.array_speed));
        end % constructor
        function obj = SetQR(obj, Q, R)
            obj.Q = Q;
            obj.R = R;
        end % setQR
        function obj = SetGainSchedule(obj, array_speed, mat_ratio)
            obj.array_speed = array_speed;
            obj.mat_ratio = mat_ratio;            
            % interp1([1,2,3],[0.1,0.2,0.3],[1.5,4], 'linear','extrap') ->
            % 0.15 0.40
        end % SetGainSchedule
        function val = control(obj, speed, h, ts, init_err, ...
                array_truck_heading_rate_d, array_trailer_heading_rate_d, ...
                max_steering_angle)
            % Gain scheduling
            gain_vec = [];
            for i=1:size(obj.mat_ratio,1)
                gain_vec = interp1(obj.array_speed, obj.mat_ratio(i,:), speed, 'spline','extrap');
            end % for
            gQ = obj.Q*blkdiag(gain_vec);
            QQ = kron(eye(h), gQ); 
            RR = kron(eye(h), obj.R);
            K = kron(eye(h), obj.model.Cd(ts));
            I = eye(h);
            for i=2:h
                K = K + kron([zeros((i-1),h);I(1:end-i+1,:)], ...
                    (obj.model.Ad(ts)^i)*obj.model.Cd(ts));
            end % for
            MM = [];
            MM_new = obj.model.Ad(ts)*init_err' + ...
                obj.model.D1d(ts, array_truck_heading_rate_d(1)) + ...
                obj.model.D2d(ts, array_trailer_heading_rate_d(1));
            MM = [MM;MM_new];
            for i=2:h
                MM_new = obj.model.Ad(ts)*MM_new + ...
                    obj.model.D1d(ts, array_truck_heading_rate_d(1)) + ...
                    obj.model.D2d(ts, array_trailer_heading_rate_d(1));
                MM = [MM;MM_new];
            end
            H = K'*QQ*K+RR;
            QQQ = K'*QQ*MM;
            [L, ~] = chol(H,'lower');
            Linv = inv(L);
            opt = mpcqpsolverOptions;
            tmp1 =  [eye(h);-eye(h)];
            tmp2 = [-ones(h,1); -ones(h,1)]*max_steering_angle;
            val = mpcqpsolver(Linv, QQQ, ...
                tmp1, tmp2, ...
                [], zeros(0,1), false(size(zeros(2*h,1))), opt);
        end % K
    end % 
end % classdef