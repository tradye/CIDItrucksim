classdef LQRController < handle
    properties
        model
        Q
        R
        tol = 0.01
        max = 150
        array_speed = [4.0, 8.0, 12.0, 20.0, 25.0];
        mat_ratio
    end % properties
    methods
        function obj = LQRController(model, tol, max)
            obj.model = model;
            obj.tol = tol;
            obj.max = max;
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
        function val = K(obj, speed, ts)
            % Gain scheduling
            gain_vec = [];
            for i=1:size(obj.mat_ratio,1)
                gain_vec = interp1(obj.array_speed, obj.mat_ratio(i,:), speed, 'spline','extrap');
            end % for
            gQ = obj.Q*blkdiag(gain_vec);
            P = gQ; 
            num_iter = 0;            
            diffSmall = realmax;
            AT = transpose(obj.model.Ad(ts));
            BT = transpose(obj.model.Cd(ts));
            A = obj.model.Ad(ts);
            B = obj.model.Cd(ts);
            while num_iter < obj.max && diffSmall > obj.tol
                P_next = AT*P*A - AT * P * B*inv(obj.R+BT*P*B) * BT * P * A + gQ;
                diffP = abs(P_next - P);
                diffCoef = max(diffP(:));
                if diffCoef < diffSmall   
                    diffSmall = diffCoef;
                end
                P = P_next;
                num_iter = num_iter + 1;
            end
            val = inv(obj.R + BT * P * B) * BT * P * A;
        end % K
    end % methods
end % classdef