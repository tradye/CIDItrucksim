clear;

% Truck Model
truckmodel = LincolnModel(); % create instance of truck

% CONSTANTS
CONST_C1 = 155494;
CONST_C2 = 155494;

% Distance of each axle to COG

CONST_b1 = 1.5;
CONST_b2 = 1.75;

% Mass of each body
CONST_m1 = 520*4;

% Inertia moment of each body
CONST_m1_f = CONST_m1/2;
CONST_m1_r = CONST_m1/2;
CONST_I1 = CONST_m1_f*CONST_b1*CONST_b1 + CONST_m1_r*CONST_b2*CONST_b2;

CONST_LON_SPEED = 10;

% Set Truck Parameters
truckmodel.set_stiffness(CONST_C1, CONST_C2);
truckmodel.set_axle_dist(CONST_b1, CONST_b2);
truckmodel.set_mass(CONST_m1);
truckmodel.set_inertia(CONST_I1);
truckmodel.set_lon_speed(CONST_LON_SPEED);

% Truck Matrices
M = truckmodel.M();
A = truckmodel.A();
C = truckmodel.C();
D1 = truckmodel.D1();

% run lqr with discrete time variab
ts = 0.01;
I = eye(truckmodel.numStates);
Ad = (I + 0.5*inv(M)*A*ts)*inv(I -0.5*inv(M)*A*ts);
Cd = inv(M)*C*ts;
D1d = inv(M)*D1*ts;

% Reform matrix A

% Call LQR Controller with Truck Model and Target Path
run('../pathtools/LoadTestPaths');

Q = blkdiag(0.0001,  0, 0.25, 0);
%Q = eye(8);
P = Q; 
num_iter = 0;
CONST_max_iterations = 150;
diffc = realmax;
tolerance = 0.01;
AT = transpose(Ad);
BT = transpose(Cd);
R = 0.015;
A = Ad;
B = Cd;
K = dlqr(Ad, Cd, Q, R, 0);
while num_iter < CONST_max_iterations && diffc > tolerance
    P_next = AT*P*A - AT * P * B*inv(R+BT*P*B) * BT * P * A + Q;
    diffm = abs(P_next - P);
    diffp = max(diffm(:));
    if diffp < diffc      
        diffc = diffp;
        mindiff = diffm;
    end
    P = P_next;
    num_iter = num_iter + 1;
end
Kt = inv(R + BT * P * B) * BT * P * A;

test = LincolnVehicle(truckmodel, pathLaneChange10);

controls = [];
xy_track = [];
for i = 1: floor(test.truck_path.cum_time_elapsed(end))/ts

    t = i*ts;
%     ind = pathLaneChange10.get_path_index_by_time(t);
%     dts  = pathLaneChange10.data.ts(ind) - pathLaneChange10.data.ts(ind-1);
%     
%     I = eye(truckmodel.numStates);
%     Ad = (I + 0.5*inv(M)*A*ts)*inv(I -0.5*inv(M)*A*ts);
%     Cd = inv(M)*C*ts;
%     D1d = inv(M)*D1*ts;
%     D2d = inv(M)*D2*ts;
%     K = dlqr(Ad, Cd, I, 1, 0);

    control = -Kt*test.get_state_error();
    act_control = test.move(control,ts);
    controls = [controls; t act_control];    

    xy_track = [xy_track; test.truck_X, test.truck_Y, ...
        test.truck_heading, test.truck_heading_rate];
end
cte1 =test.err_hist(1,:);


hold on
subplot(3,2,1);
plot(test.truck_path.data.x, test.truck_path.data.y, 'r',xy_track(:,1), xy_track(:,2),'g')
subplot(3,2,2);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle, 'r');
subplot(3,2,3);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle_rate, 'r');
subplot(3,2,4);
plot(controls(:,1),controls(:,2));
subplot(3,2,5);
x = linspace(1,length(cte1),length(cte1));
plot(x,cte1,'r');
hold off

a_track = write_path(xy_track(1:end,:), CONST_LON_SPEED);


test = LincolnVehicle(truckmodel, a_track);

controls = [];
xy_track = [];
for i = 1:  floor(test.truck_path.cum_time_elapsed(end))/ts
    t = i*ts;
%     ind = pathLaneChange10.get_path_index_by_time(t);
%     dts  = pathLaneChange10.data.ts(ind) - pathLaneChange10.data.ts(ind-1);
%     
%     I = eye(truckmodel.numStates);
%     Ad = (I + 0.5*inv(M)*A*ts)*inv(I -0.5*inv(M)*A*ts);
%     Cd = inv(M)*C*ts;
%     D1d = inv(M)*D1*ts;
%     D2d = inv(M)*D2*ts;
%     K = dlqr(Ad, Cd, I, 1, 0);

    control = -Kt*test.get_state_error();
    act_control = test.move(control,ts);
    controls = [controls; t act_control];    
    xy_track = [xy_track; test.truck_X, test.truck_Y, ...
        test.truck_heading, test.truck_heading_rate];
end
cte1 = test.err_hist(1,:);

hold on
subplot(3,2,1);
plot(test.truck_path.data.x, test.truck_path.data.y, 'r',xy_track(:,1), xy_track(:,2),'g')
subplot(3,2,2);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle, 'r');
subplot(3,2,3);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle_rate, 'r');
subplot(3,2,4);
plot(controls(:,1),controls(:,2));
subplot(3,2,5);
x = linspace(1,length(cte1),length(cte1));
plot(x,cte1,'r');
hold off

% function takes XY series and creates a path object
function path = write_path(xys, speed)
    poses = [];
    for i=1:size(xys,1)
        if mod(i,10) == 0
            poses = [poses; xys(i, 1:3)];
        end
    end
    poses(:,3) = round(rad2deg(wrapToPi(poses(:,3))/15))*15;
    pathCustom = cidiPath(poses, speed, speed, speed, 10);
    path = pathCustom;
end




% Evaluation