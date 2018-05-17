close all;
clear;

vspace = [10 20 30];
L = 3;
t1 = deg2rad(0);
diffspace = linspace(0, deg2rad(20), 200);
dt = 0.01;

actual = funcActualTrailerHeadingDelta(10, L, t1 + diffspace, t1, dt);
estimated = funcEstimatedTrailerHeadingDelta(10, L, t1 + diffspace, t1, dt);

figure;
plot(diffspace, actual, 'r', diffspace, estimated,'g')
figure;
plot(rad2deg(diffspace), 60*rad2deg(actual-estimated));
function actualHeading = funcActualTrailerHeadingDelta(v, L, t0, t1, dt)
    actualHeading = t1+v/L*sin(t0-t1)*dt;
end % funcActualTrailerHeadingDelta

function estimatedHeading = funcEstimatedTrailerHeadingDelta(v, L, t0, t1, dt)
    estimatedHeading = t1+v/L*(t0-t1)*dt;
end % funcEstimatedTrailerHeadingDelta
