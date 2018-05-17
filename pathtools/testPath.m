clear;

pathPoses = [0 0 0; 3 3 60; 5 5 60; 10 10 0; 12 10 0];
path = driving.Path.create(pathPoses, 'Reed-Shepp');

splineFitter = HelperCubicSplineFit(path);
numPoints = 1000;
splineData = fit(splineFitter, numPoints);
speedProfileGenerator = HelperSpeedProfileGenerator(splineData);
speedProfileGenerator.StartSpeed = 15;
speedProfileGenerator.EndSpeed   = 15;
speedProfileGenerator.MaxSpeed   = 15;
refSpeeds = generate(speedProfileGenerator);

v_theta = arrayfun(@(x) rad2deg(atan(splineData.dy(x)/splineData.dx(x))) ,linspace(1, numPoints, numPoints));
v_thetadot = rad2deg(transpose((refSpeeds.*splineData.kappa)));
% Plot the smoothed path
hold on
subplot(2,2,1);
x_waypoints = transpose(path.KeyPoses(1:end,1));
y_waypoints = transpose(path.KeyPoses(1:end,2));
hSmoothPath = plot(splineData.x, splineData.y, 'r', x_waypoints,y_waypoints,'o', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
xlabel('Global X Position (m)');
ylabel('Global Y Position (m)');
subplot(2,2,2);
hSmoothTheta = plot(splineData.ts, v_theta, 'b','LineWidth', 2);
xlabel('Waypoint Reference Index');
ylabel('\theta (Deg)');
subplot(2,2,3);
hSmoothThetaDot = plot(splineData.ts, v_thetadot, 'g','LineWidth', 2);
ylabel('\theta Rate (Deg/sec)');
xlabel('Waypoint Reference Index');
subplot(2,2,4);
hSpeed = plot(splineData.ts, refSpeeds, 'v', 'LineWidth', 1);
ylabel('Linear Velocity (m/s)');
xlabel('Waypoint Reference Index');
hold off

