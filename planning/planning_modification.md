# Apollo 2.5 Lattice Planner Modification for Semi-Trailer Truck

## `VehicleState`
Vehicle State needs to also incorporate trailer parameters. The current specification is under "/modules/common/vehicle_state/proto/vehicle_state.proto"

```cpp
message VehicleState {
  optional double x = 1 [default =0.0];
  optional double y = 2 [default =0.0];
  optional double z = 3 [default =0.0];
  optional double timestamp = 4 [default =0.0];
  optional double roll = 5 [default =0.0];
  optional double pitch = 6 [default =0.0];
  optional double yaw = 7 [default =0.0];
  optional double heading = 8 [default =0.0];
  optional double kappa = 9 [default =0.0];
  optional double linear_velocity = 10 [default =0.0];
  optional double angular_velocity = 11 [default =0.0];
  optional double linear_acceleration = 12 [default =0.0];
  optional apollo.canbus.Chassis.GearPosition gear = 13;
  optional apollo.canbus.Chassis.DrivingMode driving_mode = 14;
  optional apollo.localization.Pose pose = 15;
}
```
(Q: What is "`optional apollo.localization.Pose`"?)

In the case where only a hitch sensor is installed, we will need to add
```cpp
  optional hitch_angle = 16 [default =0.0];
  optional trailer_kappa = 17 [default =0.0];
```
In the case where we have a GPS/IMU device for the trailer body, replicate message items 1 through 12:
```cpp
  optional double trailer_x = 16 [default =0.0];
  optional double trailer_y = 17 [default =0.0];
  optional double trailer_z = 18 [default =0.0];
  optional double trailer_roll = 19 [default =0.0];
  optional double trailer_pitch = 20 [default =0.0];
  optional double trailer_yaw = 21 [default =0.0];
  optional double trailer_heading = 22 [default =0.0];
  optional double trailer_kappa = 23 [default =0.0];
  optional double trailer_linear_velocity = 24 [default =0.0];
  optional double trailer_angular_velocity = 25 [default =0.0];
  optional double trailer_linear_acceleration = 26 [default =0.0];
```
`VehicleState` messages need to be populated with trailer information from another module (ask gengqx or lz). Care should be taken that new fields are populated from module to module. Note that if GPS/IMU cannot be installed, then the hitch angle is sufficient to derive the remaining trailer fields. 

## Planning with New `VehicleState` Message in `planning.cc`
Planning is initiated by the function in `lattice_planner.cc` (/modules/planning/planner/lattice/lattice_planner.cc)
```cpp
	common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
```
Here the `Frame` object contains a call to obtain the `VehicleState`:
```cpp
	const common::VehicleState &Frame::vehicle_state() const {
		return vehicle_state_;
	}
```
**The module has two new tasks**:
1. For each candidate trajectory in the (combined) trajectory bundle, use a model to generate a trailer trajectory. Currently, I use a kinematic model. Later, we can use a linear dynamic model with a regression-based steering calculation, or a nonlinear dynamic model.

2. For each pair of truck-trailer trajectories, check collision conditions by generating a dynamic collision path.

## 1. `TrailerTrajectoryGenerator.cc`
```cpp


```
An additional input can be considered to allow for different prediction models for the trailer body:

```cpp
enum TrailerPredictionMode {
	DOUBLE = 0,
	KINEMATIC = 1,
	DYNAMIC = 2,
};
```


## 2. `TruckCollisionChecker.cc`

## Other Possible Class Modifications:



## 3. Collision Checking Without Hitch Angle
- **Plan 1**: Estimating hitch angle using Kalman Filters in VehicleStateProvider
--Requires heavy modification to VehicleStateProvider
--Requires knowing some previous path information. May be hard to keep track of.
- **Plan 2**: Estimating hitch angle using Kalman Filters in the trailer trajectory generator.
--Planning module requires knowing previous previous path information. Easier to keep track of.
--Does not solve the problem of hitch angle remaining unestimated for the control module. Solution: do not optimize the trailer angle.
- **Plan 3**: Use large safety boundaries for collision checks. The safety boundaries should evolve according to the generated truck trajectory. However, we still need an estimate for the hitch angle.
-- Does not solve the problem of hitch angle remaining unestimated for the control module. Solution: do not optimize the trailer angle.
--(Because a planning estimate for the hitch angle is needed, plans 2 and 3 really should be combined into one plan.)
- **Plan 4**: Lastly, an idea is to directly generate a box that keeps track of the previous path of the trailer (8 seconds of previous trajectory, based on the published trajectory). This previous trajectory provides a previous kappa and a heading. We need to intialize or calibrate these values on module start.
-- Write a third class that always keeps track of the published trajectory and "provides" a heading and kappa to TrailerTrajectoryGenerator! This class is also responsible for initialization and calibration.
