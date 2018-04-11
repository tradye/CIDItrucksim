# CIDItrucksim
simulation using trucksim and matlab

# M file

the main algorithm comes from apollo:

ComputeCOMPosition.m:comptuing COG based on the current truck location

computeFeedForward.m:computing feedforward angle 

ComputeLateralErrors.m:computing lateral errors

QueryNearestPointByPosition.m:querying nearest point index

readPlanPoints.m:getting the planning results

SolveLQRProblem.m:LQR algorithm

Interpolate.m:linear interpolate of linear velocity and matrix Q

truck_LQRController.m:the main file of s-function

