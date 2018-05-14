clear;

% Truck Model
truckmodel = TruckModel6Axle(); % create instance of truck

% CONSTANTS
CONST_C1 = 155494*2;
CONST_C2 = 155494*4;
CONST_C3 = 155494*4;
CONST_C4 = 155494*4;
CONST_C5 = 155494*4;
CONST_C6 = 155494*4;

% Distance of each axle to COG
CONST_TRUCK_COGFRAC = 0.5;
CONST_TRAIL_COG_FRAC = 0.15;
CONST_b1 = 3.9 * CONST_TRUCK_COGFRAC;
CONST_b2 = 3.9 * (1-CONST_TRUCK_COGFRAC);
CONST_b3 = CONST_b2+1.310;

CONST_b4 = 4.4*(1-CONST_TRAIL_COG_FRAC);
CONST_b5 = CONST_b4+1.310;
CONST_b6 = CONST_b5+1.310;

% Mass of each body
CONST_m1 = 8800;
CONST_m2 = 32000;

% Distance of hitch to COG for each body
CONST_hF = CONST_b2 + 0.075;
CONST_hR = 7.16-4.4*(1-CONST_TRAIL_COG_FRAC); % approximation of hitch-COG distance of trailer

% Inertia moment of each body
CONST_m1_f = CONST_m1/2;
CONST_m1_r = CONST_m1/2;
CONST_m2_f = CONST_m2*0.25;
CONST_m2_r = CONST_m2*0.75; % assume the rear wheelbases carry 75% of the weight
CONST_I1 = CONST_m1_f*CONST_b1*CONST_b1 + CONST_m1_r*CONST_hF*CONST_hF;
CONST_I2 = CONST_m2_f*CONST_hR*CONST_hR + CONST_m2_r*CONST_b5*CONST_b5;

CONST_LON_SPEED = 30;

% Set Truck Parameters
truckmodel.set_stiffness(CONST_C1, CONST_C2, CONST_C3, CONST_C4, CONST_C5, CONST_C6); 
truckmodel.set_axle_dist(CONST_b1, CONST_b2, CONST_b3, CONST_b4, CONST_b5, CONST_b6);
truckmodel.set_hitch_dist(CONST_hF, CONST_hR);
truckmodel.set_mass(CONST_m1, CONST_m2);
truckmodel.set_inertia(CONST_I1, CONST_I2);
truckmodel.set_lon_speed(CONST_LON_SPEED);

% Truck Matrices
M = truckmodel.M();
A = truckmodel.A();
C = truckmodel.C();
D1 = truckmodel.D1();
D2 = truckmodel.D2();

% run lqr with discrete time variab
ts = 0.01;
I = eye(truckmodel.num_states);
Ad = (I + 0.5*inv(M)*A*ts)*inv(I -0.5*inv(M)*A*ts);
Cd = inv(M)*C*ts;
D1d = inv(M)*D1*ts;
D2d = inv(M)*D2*ts;

% Reform matrix A

% Call LQR Controller with Truck Model and Target Path
run('../pathtools/LoadTestPaths');

Q = blkdiag(0,  0, 0.5, 0, 0, 0, 0.5, 0);
%Q = eye(8);
P = Q; 
num_iter = 0;
CONST_max_iterations = 1500;
diffc = realmax;
tolerance = 0.01;
AT = transpose(Ad);
BT = transpose(Cd);
R = 0.015;
A = Ad;
B = Cd;
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
% K = dlqr(Ad, Cd, Q, R, 0);

InitPose = [0;0;0;0;0;0;0;0];
test = TruckVehicle(truckmodel, PathLCFast);

controls = [];
xy_track = [];
xy_trailer = [];
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
    xy_trailer = [xy_trailer; test.trailer_X, test.trailer_Y, ...
        test.trailer_heading, test.trailer_heading_rate];
end
cte1 =test.err_hist(1,:);
cte2 = test.err_hist(5,:);


hold on
subplot(3,2,1);
plot(test.truck_path.data.x, test.truck_path.data.y, 'r', test.trailer_path.x, test.trailer_path.y, 'b',xy_track(:,1), xy_track(:,2),'g')
subplot(3,2,2);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle, 'r', test.truck_path.cum_time_elapsed, test.trailer_path.angle, 'b');
subplot(3,2,3);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle_rate, 'r', test.truck_path.cum_time_elapsed, test.trailer_path.angle_rate, 'b');
subplot(3,2,4);
plot(controls(:,1),controls(:,2));
subplot(3,2,5);
x = linspace(1,length(cte1),length(cte1));
plot(x,cte1,'r',x,cte2,'b');
hold off

a_track = write_path(xy_track(1:end,:), CONST_LON_SPEED);
b_track = write_path(xy_trailer(1:end,:), CONST_LON_SPEED);

test = TruckVehicle(truckmodel, a_track, b_track);

controls = [];
xy_track = [];
xy_trailer = [];
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
    xy_trailer = [xy_trailer; test.trailer_X, test.trailer_Y, ...
        test.trailer_heading, test.trailer_heading_rate];
end
cte1 = test.err_hist(1,:);
cte2 = test.err_hist(5,:);
hold on
subplot(3,2,1);
plot(test.truck_path.data.x, test.truck_path.data.y, 'r', test.trailer_path.x, test.trailer_path.y, 'b',xy_track(:,1), xy_track(:,2),'g', xy_trailer(:,1), xy_trailer(:,2), 'k')
subplot(3,2,2);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle, 'r', test.truck_path.cum_time_elapsed, test.trailer_path.angle, 'b');
subplot(3,2,3);
plot(test.truck_path.cum_time_elapsed, test.truck_path.angle_rate, 'r', test.truck_path.cum_time_elapsed, test.trailer_path.angle_rate, 'b');
subplot(3,2,4);
plot(controls(:,1),controls(:,2));
subplot(3,2,5);
x = linspace(1,length(cte1),length(cte1));
plot(x,cte1,'r',x,cte2,'b');
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