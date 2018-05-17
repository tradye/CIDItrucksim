# CiDi Truck Model

此Git是为了让平台团队在仿真卡车挂车模型分享代码。Access容许设为Private。
Repo管理人员：胡斯博

## Matlab Files
- `testLQR.m`: test class that tests LQR. Note that we run LQR twice: once to generate a trailer path, and again to calculate the actual controls
- `testMPC.m`: test class that tests MPC. Again we run MPC twice, first to generate a tralier reference path
- `testLQRMPC.m`: test class that generates a trailer path using LQR and controls using MPC.
- `LoadTestPaths.m`: a routine that creates several sample cidiPath objects for control testing
- `cidiPath.m`: a object class that abstracts away the Pose-Path MATLAB procedure

## Main Classes
### Truck-Trailer Model
- `TruckModel6Axle.m`: Model class for the 6 axle semi-trailer truck.
- `TruckVehicle.m`: Simulation truck vehicle class that takes a control command calculates a vehicle position. This is there only to test the controllers.

### Lincoln Model
- `LincolnModel.m`: Model class for the Lincoln vehicle
- `TruckVehicle.m`: Simulation Lincoln vehicle class that takes a control command calculates a vehicle position. This is there only to test the controllers

## Controllers
- `LQRController.m`: This is the LQR controller class used to calculate a K matrix. Requires a model, tolerance, and max iteration count
- `MPCController.m`: This is the MPC controller class used to calculate a control command. Requires a truck model

### Helper Files:
- `HelperCubicSplineFit.m` is from the MATLAB Automated Valet Parking Example
- `HelperSpeedProfileGenerator.m` is from the MATLAB Automated Valet Parking Example
