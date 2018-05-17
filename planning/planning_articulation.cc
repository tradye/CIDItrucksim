/**
 * Copyright 2018 CiDi
 **/

#include "modules/planning/lattice/trajectory_generation/planning_articulation.h"


#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

 double PlanningArticulation::GetCurrentHitchAngle(Frame* frame) {
  if (!is_last_trajectory_set_) {
   return 0.0
  }
  // last trajectories are set, hitch angle is the trailer angle minus the truck angle
  std::uint32_t index = GetCurrentTrajectoryIndex(frame);
  return GetCurrentTrailerHeading(index) - frame->vehicle_state().heading(); 
 }

 
 double PlanningArticulation::GetCurrentTrailerHeading(Frame* frame) {
  if (!is_last_trajectory_set_) {
   return frame->vehicle_state().heading();
  }
  std::uint32_t index = GetCurrentTrajectoryIndex(frame);
  TrajectoryPoint trailer_trajectory_point =
   last_discretized_trailer_trajectory_.TrajectoryPointAt(index);
  return trailer_trajectory_point.path_point().theta();
 }

 
  double PlanningArticulation::GetCurrentTrailerKappa(Frame* frame) {
  if (!is_last_trajectory_set_) {
   return frame->vehicle_state().kappa();
  }
  std::uint32_t index = GetCurrentTrajectoryIndex(frame);
  TrajectoryPoint trailer_trajectory_point =
   last_discretized_trailer_trajectory_.TrajectoryPointAt(index);
  return trailer_trajectory_point.path_point().kappa();
 }

 
 std::uint32_t GetCurrentTrajectoryIndex(Frame* frame) {
  if (is_last_trajectory_set_) {  
  // get truck position on last trajectory: frame 
  const common::math::Vec2D frame_position(frame->vehicle_state().x(),				  
					   frame->vehicle_state().y()
					   );
  current_trajectory_index_ =
    last_discretized_truck_trajectory_.QueryNearestPoint(frame_position);
  }
}

 
 void PlanningArticulation::SetTrajectories(DiscretizedTrajectory discretized_truck_trajectory,
					    DiscretizedTrajectory discretized_trailer_trajectory) {
  last_discretized_truck_trajectory_ = discretized_truck_trajectory;
  last_discretized_trailer_trajectory_ = discretized_trailer_trajectory;
  is_last_trajectory_set_ = true;
 }
 
}  // namespace planning
}  // namespace apollo
