clear;

run('LoadTestPaths.m')

targetPath = pathLCSlow;

hR = 3;
hF = 3.42;
statevar = []; % x0 y0 theta0 x1 y1 theta1
theta0 = 0;
theta1 = 0;
for i=1:length(targetPath.x)
    x0 = targetPath.x(i);
    y0 = targetPath.y(i);    
    v0 = targetPath.speedProfile(i);    
    theta1 = theta1 + v0/hF* sin(theta0 - theta1)* ...
        targetPath.time_elapsed(i);
    theta0 = targetPath.angle(i);
    x1 = x0 - hR*cos(theta0) - hF*cos(theta1);
    y1 = y0 - hR*sin(theta0) - hF*sin(theta1);
    diff = sqrt((y1-y0)^2 + (x1-x0)^2);    
    statevar = [statevar; x0 y0 theta0 x1 y1 theta1 diff];
end % for i 
% plot(statevar(:,1), statevar(:,2),'r',statevar(:,4),statevar(:,5),'b');
plot(statevar(:,1), statevar(:,3),'r',statevar(:,4),statevar(:,6),'b');

% plot(diff,'r');