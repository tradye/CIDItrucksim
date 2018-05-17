classdef TruckModel6Axle < matlab.mixin.Copyable
    properties
        m1 = 1 % Truck mass
        m2 = 1 % Trailer mass
        C1 = 1 % Truck Axle 1 Cornering Stiffness
        C2 = 1 % Truck Axle 2 Cornering Stiffness
        C3 = 1 % Truck Axle 3 Cornering Stiffness
        C4 = 1 % Trailer Axle 4 Cornering Stiffness
        C5 = 1 % Trailer Axle 5 Cornering Stiffness
        C6 = 1 % Trailer Axle 6 Cornering Stiffness
        b1 = 1 % Truck Axle 1 Distance to COG
        b2 = 1 % Truck Axle 2 Distance to COG
        b3 = 1 % Truck Axle 3 Distance to COG
        b4 = 1 % Trailer Axle 4 Distance to COG
        b5 = 1 % Trailer Axle 5 Distance to COG
        b6 = 1 % Trailer Axle 6 Distance to COG
        hF = 1 % Hitch Distance to Truck COG
        hR = 1 % Hitch Distance to Trailer COG
        I1 = 1 % Truck Inertia Moment
        I2 = 1 % Trailer Inertia Moment
        Vx = 1 % Linear Velocity
        numStates = 8
    end % properties    
    methods
        function out = M(obj)
            mat1 = [1, 0, 0, 0, 0, 0, 0, 0];
            mat2 = [0, obj.m1, 0, 0, 0, obj.m2, 0, 0];
            mat3 = [0, 0, 1, 0, 0, 0, 0, 0];
            mat4 = [0, 0, 0, obj.I1, 0, -obj.hF*obj.m2, 0, 0];
            mat5 = [0, 0, 0, 0, 1, 0, 0, 0];
            mat6 = [0, 0, 0, 0, 0, -obj.hR*obj.m2, 0, obj.I2];
            mat7 = [0, 0, 0, 0, 0, 0, 1, 0];
            mat8 = [0, 1, 0, -obj.hF, 0, -1, 0, -obj.hR];
            out = [mat1;mat2;mat3;mat4;mat5;mat6;mat7;mat8];
        end % M
        function out = A(obj)
            ea = (-obj.C1-obj.C2-obj.C3)/obj.Vx;
            eb = obj.C1+obj.C2+obj.C3;
            ec = (-obj.b1*obj.C1+obj.b2*obj.C2+obj.b3*obj.C3)/obj.Vx;

            ed = (-obj.C4-obj.C5-obj.C6)/obj.Vx;
            ee = obj.C4+obj.C5+obj.C6;
            ef = (obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)/obj.Vx;

            eg = (-obj.b1*obj.C1+obj.b2*obj.C2+obj.b3*obj.C3)/obj.Vx;
            eh = obj.b1*obj.C1-obj.b2*obj.C2-obj.b3*obj.C3;
            ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2)-obj.C3*(obj.b3^2))/obj.Vx;

            ej = (obj.hF*obj.C4+obj.hF*obj.C5+obj.hF*obj.C6)/obj.Vx;
            ek = -obj.hF*obj.C4-obj.hF*obj.C5-obj.hF*obj.C6;
            el = obj.hF*(-obj.b4*obj.C4-obj.b5*obj.C5-obj.b6*obj.C6)/obj.Vx;

            em = (obj.hR*(obj.C4+obj.C5+obj.C6) + obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)/obj.Vx;
            en = -obj.hR*(obj.C4+obj.C5+obj.C6)-(obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6);
            eo = (-obj.hR*(obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)-(obj.C4*(obj.b4^2)+obj.C5*(obj.b5^2)+obj.C6*(obj.b6^2)))/obj.Vx;
           
            mat1 = [0, 1, 0, 0, 0, 0, 0, 0];
            mat2 = [0, ea, eb, ec, 0, ed, ee, ef];
            mat3 = [0 ,0 , 0, 1, 0, 0, 0, 0];
            mat4 = [0, eg, eh, ei, 0, ej, ek, el];
            mat5 = [0, 0, 0, 0, 0, 1, 0, 0];
            mat6 = [0, 0, 0, 0, 0, em, en, eo];
            mat7 = [0, 0, 0, 0, 0, 0, 0, 1];
            mat8 = [0, 0, 0, 0, 0, 0, 0, 0];
            out = [mat1;mat2;mat3;mat4;mat5;mat6;mat7;mat8];
        end % A
        function out = C(obj)
            out = [0; obj.C1; 0; obj.b1*obj.C1; 0 ; 0; 0; 0];
        end % C
        function out = D1(obj)
            eg = (-obj.b1*obj.C1+obj.b2*obj.C2+obj.b3*obj.C3)/obj.Vx;            
            ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2)-obj.C3*(obj.b3^2))/obj.Vx; 
            out = [0; eg - obj.m1*obj.Vx; 0; ei; 0; 0; 0; -obj.Vx];
        end % D1
        function out = D2(obj)
            ef = (obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)/obj.Vx;
            el = obj.hF*(-obj.b4*obj.C4-obj.b5*obj.C5-obj.b6*obj.C6)/obj.Vx;
            eo = (-obj.hR*(obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)-(obj.C4*(obj.b4^2)+obj.C5*(obj.b5^2)+obj.C6*(obj.b6^2)))/obj.Vx;
            out = [0; ef - obj.m2*obj.Vx; 0; el + obj.hF*obj.m2*obj.Vx; 0; eo+obj.hR*obj.m2*obj.Vx; 0; obj.Vx];            
        end % D2
        function out = LA(obj)
        % 4x4 Matrix A in Levels
        ea = (-obj.C1-obj.C2-obj.C3)/obj.Vx;
        eb = obj.C1+obj.C2+obj.C3;
        ec = (-obj.b1*obj.C1+obj.b2*obj.C2+obj.b3*obj.C3)/obj.Vx;

        ed = (-obj.C4-obj.C5-obj.C6)/obj.Vx;
        ee = obj.C4+obj.C5+obj.C6;
        ef = (obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)/obj.Vx;

        eg = (-obj.b1*obj.C1+obj.b2*obj.C2+obj.b3*obj.C3)/obj.Vx;
        eh = obj.b1*obj.C1-obj.b2*obj.C2-obj.b3*obj.C3;
        ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2)-obj.C3*(obj.b3^2))/obj.Vx;

        ej = (obj.hF*obj.C4+obj.hF*obj.C5+obj.hF*obj.C6)/obj.Vx;
        ek = -obj.hF*obj.C4-obj.hF*obj.C5-obj.hF*obj.C6;
        el = obj.hF*(-obj.b4*obj.C4-obj.b5*obj.C5-obj.b6*obj.C6)/obj.Vx;

        em = (obj.hR*(obj.C4+obj.C5+obj.C6) + obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)/obj.Vx;
        en = -obj.hR*(obj.C4+obj.C5+obj.C6)-(obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6);
        eo = (-obj.hR*(obj.b4*obj.C4+obj.b5*obj.C5+obj.b6*obj.C6)-(obj.C4*(obj.b4^2)+obj.C5*(obj.b5^2)+obj.C6*(obj.b6^2)))/obj.Vx;
           
        mat1 = [ea, ec - obj.m2*obj.Vx, ed, ef-obj.m2*obj.Vx];
        mat2 = [eg, ei, ej, el+obj.hF*obj.m2*obj.Vx];
        mat3 = [0, 0, em, eo + obj.hR*obj.m2*obj.Vx];
        mat4 = [0, 0, 0, 0];
        out = [mat1;mat2;mat3;mat4];
        end % LA
        function out = LC(obj)
        % 4x1 Matrix C in Levels
            out = [obj.C1;obj.C1*obj.b1;0;0];
        end % LC
        function out = LM(obj)
            mat1 = [obj.m1, 0, obj.m2, 0];
            mat2 = [0, obj.I1, -obj.hF*obj.m2, 0];
            mat3 = [0, 0, -obj.hR*obj.m2, obj.I2];
            mat4 = [1, -obj.hF, -1, -obj.hR];
            out = [mat1;mat2;mat3;mat4];
        end % LM
        function obj = set_stiffness(obj, C1, C2, C3, C4, C5, C6)
            obj.C1 = C1;
            obj.C2 = C2;
            obj.C3 = C3;
            obj.C4 = C4;
            obj.C5 = C5;
            obj.C6 = C6;            
        end %set_stiffnesses
        function obj = set_mass(obj, m1, m2)
            obj.m1 = m1;
            obj.m2 = m2;          
        end %set_mass
        function obj = set_inertia(obj, I1, I2)
            obj.I1 = I1;
            obj.I2 = I2;            
        end %set_inertia
        function obj = set_axle_dist(obj, b1, b2, b3, b4, b5, b6)
            obj.b1 = b1;
            obj.b2 = b2;
            obj.b3 = b3;
            obj.b4 = b4;
            obj.b5 = b5;
            obj.b6 = b6;
        end % set_axle_dist
        function obj = set_hitch_dist(obj, hF, hR)
            obj.hF = hF;
            obj.hR = hR;
        end % set_hitch_dist
        function obj = set_lon_speed(obj, vx)
            obj.Vx = vx;
        end % set_lon_speed
        function val = Ad(obj,ts)
            val = (eye(obj.numStates) + 0.5*inv(obj.M)*obj.A*ts)*inv(eye(obj.numStates) - 0.5*inv(obj.M)*obj.A*ts);
        end
        function val = Cd(obj, ts)
            val = inv(obj.M)*obj.C*ts;
        end
        function val = D1d(obj, ts, heading_rate_d)
            val = inv(obj.M)*obj.D1*ts*heading_rate_d;
        end
        function val = D2d(obj, ts, heading_rate_d)
            val = inv(obj.M)*obj.D2*ts*heading_rate_d;
        end
    end
end %classdef