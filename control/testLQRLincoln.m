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
% Set Truck Parameters
truckmodel.set_stiffness(CONST_C1, CONST_C2);
truckmodel.set_axle_dist(CONST_b1, CONST_b2);
truckmodel.set_mass(CONST_m1);
truckmodel.set_inertia(CONST_I1);

% run lqr with discrete time variab
ts = 0.01;
% Call LQR Controller with Truck Model and Target Path
run('LoadTestPaths');

CONST_TOL = 0.01;
CONST_MAXITER = 150;
controller = LQRController(truckmodel, CONST_TOL, CONST_MAXITER);
Q = blkdiag(0.0005,  0, 0.25, 0);
R = 0.015;
controller.SetQR(Q, R);
gain_speeds = [4.0, 8.0, 12.0, 20.0, 25.0];
lateral_gain = [1.0, 0.6, 0.2, 0.1, 0.05];
heading_gain = [1.0, 0.6, 0.4, 0.2, 0.1];
controller.SetGainSchedule(gain_speeds, ...
    [lateral_gain; ones(1,length(gain_speeds)); ...
    heading_gain; ones(1,length(gain_speeds))]);

CONST_LON_SPEED = 10;
truckmodel.set_lon_speed(CONST_LON_SPEED);
test = LincolnVehicle(truckmodel, pathLCSlow);

test.run(controller, ts);

a_track = test.get_truck_cidipath();

test = LincolnVehicle(truckmodel, a_track);

test.run(controller, ts);
test.plot();