classdef LincolnModel < matlab.mixin.Copyable
    properties
        m1 = 1 % Car mass
        C1 = 1 % Car Axle 1 Cornering Stiffness
        C2 = 1 % Car Axle 2 Cornering Stiffness
        b1 = 1 % Car Axle 1 Distance to COG
        b2 = 1 % Car Axle 2 Distance to COG
        I1 = 1 % Car Inertia Moment
        Vx = 1 % Linear Velocity
        numStates = 4
    end % properties    
    methods
        function out = M(obj)
            mat1 = [1, 0, 0, 0];
            mat2 = [0, obj.m1, 0, 0];
            mat3 = [0, 0, 1, 0];
            mat4 = [0, 0, 0, obj.I1];
            out = [mat1;mat2;mat3;mat4];
        end % M
        function out = A(obj)
            ea = (-obj.C1-obj.C2)/obj.Vx;
            eb = obj.C1+obj.C2;
            ec = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.Vx;

            eg = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.Vx;
            eh = obj.b1*obj.C1-obj.b2*obj.C2;
            ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2))/obj.Vx;
           
            mat1 = [0, 1, 0, 0];
            mat2 = [0, ea, eb, ec];
            mat3 = [0 ,0 , 0, 1];
            mat4 = [0, eg, eh, ei];
            out = [mat1;mat2;mat3;mat4];
        end % A
        function out = C(obj)
            out = [0; obj.C1; 0; obj.b1*obj.C1;];
        end % C
        function out = D1(obj)
            eg = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.Vx;            
            ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2))/obj.Vx; 
            out = [0; eg - obj.m1*obj.Vx; 0; ei];
        end % D1
        function out = LA(obj)
        % 4x4 Matrix A in Levels
        ea = (-obj.C1-obj.C2)/obj.Vx;
        ec = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.Vx;
        eg = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.Vx;
        ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2))/obj.Vx;      
        mat1 = [ea, ec - obj.m2*obj.Vx];
        mat2 = [eg, ei];
        out = [mat1;mat2];
        end % LA
        function out = LC(obj)
        % 4x1 Matrix C in Levels
            out = [obj.C1;obj.C1*obj.b1];
        end % LC
        function out = LM(obj)
            mat1 = [obj.m1, 0];
            mat2 = [0, obj.I1];
            out = [mat1;mat2];
        end % LM
        function obj = set_stiffness(obj, C1, C2)
            obj.C1 = C1;
            obj.C2 = C2;
        end %set_stiffnesses
        function obj = set_mass(obj, m1)
            obj.m1 = m1;          
        end %set_mass
        function obj = set_inertia(obj, I1)
            obj.I1 = I1;
        end %set_inertia
        function obj = set_axle_dist(obj, b1, b2)
            obj.b1 = b1;
            obj.b2 = b2;
        end % set_axle_dist
        function obj = set_lon_speed(obj, vx)
            obj.Vx = vx;
        end % set_lon_speed
        function val = Ad(obj,ts)
            val = (eye(obj.numStates) + 0.5*inv(obj.M)*obj.A*ts)*inv(eye(obj.numStates) - 0.5*inv(obj.M)*obj.A*ts);
        end
        function val = Cd(obj, ts)
            val = inv(obj.M)*obj.C*ts;
        end
    end
end %classdef