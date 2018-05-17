/**
 * Copyright 2018 CiDi
 **/

#include "modules/planning/lattice/trajectory_generation/trailer_trajectory_generator.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {
  
using apollo::common::PathPoint;
using common::TrajectoryPoint;

TrailerTrajectoryGenerator::TrailerTrajectoryGenerator() {
	const auto& vehicle_config =
		common::VehicleConfigHelper::instance()->GetConfig();
	length_truck_cog_to_hitch_ = vehicle_config.length_truck_cog_to_hitch();
	length_trailer_cog_to_hitch_ = vehicle_config.length_trailer_cog_to_hitch();		
}

// TODO: Add DEBUG messages
DiscretizedTrajectory TrailerTrajectoryGenerator::Generate(const DiscretizedTrajectory
							    truck_trajectory,
							    Frame* frame) {
  const auto& vehicle_config =
   common::VehicleConfigHelper::instance()->GetConfig();
  double length_truck_cog_to_hitch_ = vehicle_config.length_truck_cog_to_hitch();
  double length_trailer_cog_to_hitch_ = vehicle_config.length_trailer_cog_to_hitch();
	
  // frame contains information on current vehicle state
	
  DiscretizedTrajectory trailer_trajectory;
  std::uint32_t i_param = 1;
	   	
  TrajectoryPoint prev_truck_trajectory_point;
  TrajectoryPoint prev_trailer_trajectory_point;
  TrajectoryPoint curr_truck_trajectory_point;
  TrajectoryPoint curr_trailer_trajectory_point;	
			
  double curr_truck_theta;
  double prev_truck_theta;
  double curr_trailer_theta;
  double prev_trailer_theta;

  double curr_linear_velocity;
  double prev_linear_velocity;

  // Iterate through truck_trajectory
  while (i_param < truck_trajectory.NumOfPoints()) {
   // Use kinematic model to populate trailer_trajectory
		
   double linear_acceleration;		
   double trailer_angular_velocity;
   double trailer_kappa;
		
   double dt;
			
   curr_truck_trajectory_point = truck_trajectory.TrajectoryPointAt(i_param);	
		
   double dt;
   if (i_param == 0) { 
    // initially, get trailer angle from frame (vehicle state)
    dt = 0.0;
			
    double hitch_angle = frame->vehicle_state().hitch_angle();
			
    curr_truck_theta = frame->vehicle_state().heading();
    curr_trailer_theta = curr_truck_theta + curr_hitch_theta;
			
    curr_linear_velocity = frame->vehicle_state().linear_velocity();
    linear_acceleration = frame->vehicle_state().linear_acceleration();
    trailer_kappa = frame->vehicle_state().trailer_kappa();
			
   } else { 
    // past the first point, use kinematic model to predict
    dt = curr_truck_trajectory_point.relative_time() -
     prev_truck_trajectory_point.relative_time();
			
    curr_truck_theta = curr_truck_trajectory_point.path_point().theta();
    prev_truck_theta = prev_truck_trajectory_point.path_point().theta();
    // curr_trailer_theta to be calculated
    prev_trailer_theta = prev_trailer_trajectory_point.path_point().theta();
			
    curr_linear_velocity = curr_truck_trajectory_point.v();
    prev_linear_velocity = prev_truck_trajectory_point.v();			
    linear_acceleration = (curr_linear_velocity - prev_linear_velocity)/dt;

    // main equation of motion
    curr_trailer_theta = prev_trailer_theta + 
     prev_linear_velocity/length_trailer_cog_to_hitch_ * 
     sin(prev_truck_theta - prev_trailer_theta) * dt;
    trailer_angular_velocity = AngleDiff(curr_trailer_theta,
					 prev_trailer_theta)/dt;
    trailer_kappa = trailer_angular_velocity/curr_linear_velocity;
   }		
   const double cos_truck_theta = std::cos(curr_truck_theta);
   const double cos_trailer_theta = std::cos(curr_trailer_theta);
   const double sin_truck_theta = std::sin(curr_truck_theta);
   const double sin_trailer_theta = std::sin(curr_trailer_theta);

   double trailer_x = curr_truck_trajectory_point.path_point().x() -
    length_truck_cog_to_hitch_ * cos_truck_theta -
    length_trailer_cog_to_hitch_ * cos_trailer_theta;
   double trailer_y = curr_truck_point.path_point().y() - 
    length_truck_cog_to_hitch_ * sin_truck_theta -
    length_trailer_cog_to_hitch_ * sin_trailer_theta;
		
   TrajectoryPoint curr_trailer_trajectory_point;
   curr_trailer_trajectory_point.mutable_path_point()->set_x(trailer_x);
   curr_trailer_trajectory_point.mutable_path_point()->set_y(trailer_y);
   curr_trailer_trajectory_point.mutable_path_point()->set_s();
   curr_trailer_trajectory_point.mutable_path_point()->set_theta(curr_trailer_theta);
   curr_trailer_trajectory_point.mutable_path_point()->set_kappa(trailer_kappa);
   curr_trailer_trajectory_point.set_v(curr_linear_velocity);
   curr_trailer_trajectory_point.set_a(linear_acceleration);
   curr_trailer_trajectory_point.set_relative_time(truck_relative_time);
   trailer_trajectory.AppendTrajectoryPoint(curr_trailer_trajectory_point);		

   prev_truck_trajectory_point = curr_truck_trajectory_point;
   prev_trailer_trajectory_point = curr_trailer_trajectory_point;
 
  }
  return trailer_trajectory;
}

}  // namespace planning
}  // namespace apollo
