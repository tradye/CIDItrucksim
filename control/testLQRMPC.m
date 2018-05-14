close all;
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
CONST_TRUCK_COGFRAC = 0.25;
CONST_TRAIL_COG_FRAC = 0.15;
CONST_b1 = 3.9 * CONST_TRUCK_COGFRAC;
CONST_b2 = 3.9 * (1-CONST_TRUCK_COGFRAC);
CONST_b3 = CONST_b2+1.310;

CONST_b4 = 4.4*(1-CONST_TRAIL_COG_FRAC);
CONST_b5 = CONST_b4+1.310;
CONST_b6 = CONST_b5+1.310;

% Mass of each body
CONST_m1 = 8800;
CONST_m2 = 30000;

% Distance of hitch to COG for each body
CONST_hF = CONST_b2 + 0.075;
CONST_hR = 7.16-4.4*(1-CONST_TRAIL_COG_FRAC); % approximation of hitch-COG distance of trailer

% Inertia moment of each body
CONST_m1_f = CONST_m1*0.75;
CONST_m1_r = CONST_m1*0.25;
CONST_m2_f = CONST_m2*0.25;
CONST_m2_r = CONST_m2*0.75; % assume the rear wheelbases carry 75% of the weight
CONST_I1 = CONST_m1_f*CONST_b1*CONST_b1 + CONST_m1_r*CONST_hF*CONST_hF;
CONST_I2 = CONST_m2_f*CONST_hR*CONST_hR + CONST_m2_r*CONST_b5*CONST_b5;

CONST_LON_SPEED = 20;

% Set Truck Parameters
truckmodel.set_stiffness (CONST_C1, CONST_C2, CONST_C3, CONST_C4, CONST_C5, CONST_C6); 
truckmodel.set_axle_dist(CONST_b1, CONST_b2, CONST_b3, CONST_b4, CONST_b5, CONST_b6);
truckmodel.set_hitch_dist(CONST_hF, CONST_hR);
truckmodel.set_mass(CONST_m1, CONST_m2);
truckmodel.set_inertia(CONST_I1, CONST_I2);
truckmodel.set_lon_speed(CONST_LON_SPEED);
% run lqr with discrete time variab
ts = 0.01;
% Call LQR Controller with Truck Model and Target Path
run('LoadTestPaths');

CONST_TOL = 0.01;
CONST_MAXITER = 150;
controller = LQRController(truckmodel, CONST_TOL, CONST_MAXITER);
Q = blkdiag(0,  0.02, 0.05, 0, 0, 0.02, 1, 0);
R = 1;
controller.SetQR(Q, R);
gain_speeds = [4.0, 8.0, 12.0, 20.0, 25.0];
lateral_gain = [0.8, 0.6, 0.2, 0.1, 0.05];
heading_gain = [1.0, 0.6, 0.3, 0.1, 0.1];
controller.SetGainSchedule(gain_speeds, ...
    [lateral_gain; ones(1,length(gain_speeds)); ...
    heading_gain; ones(1,length(gain_speeds)); ...
    lateral_gain; ones(1, length(gain_speeds)); ...
    heading_gain; ones(1, length(gain_speeds)) ...
    ]);
test = TruckVehicle(truckmodel, pathLCFast);

test.run(controller, ts);
figure
test.plot()

a_track = test.get_truck_cidipath();
b_track = test.get_trailer_cidipath();


test = TruckVehicle(truckmodel, a_track, "explicit", b_track);

Q = blkdiag(1,  0.5, 1, 0, 0, 0, 1, 0);
R = 1;
controller.SetQR(Q, R);

test.run(controller, ts);
figure
test.plot();
