/**
 * Copyright 2018 CiDi
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_PLANNING_ARTICULATION_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_PLANNING_ARTICULATION_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

 // What does this class do?
 // 
 
class PlanningArticulation {
 public:
  double GetCurrentHitchAngle(Frame* frame);
  
  double GetCurrentTrailerHeading(Frame* frame);
  
  double GetCurrentTrailerKappa(Frame* frame);

  std::uint32_t GetCurrentTrajectoryIndex(Frame* frame);

  bool SetTrajectories(DiscretizedTrajectory discretized_truck_trajectory,
		       DiscretizedTrajectory discretized_trailer_trajectory);
  
 private:  
  DiscretizedTrajectory last_discretized_truck_trajectory_;
  
  DiscretizedTrajectory last_discretized_trailer_trajectory_;
  
  bool is_last_trajectory_set_ = false;
  
  std::uint32_t current_trajectory_index_ = 0;
  common::math::Vec2d current_position_ = common::math::Vec2d();
};

}  // namespace planning
}  // namespace apollo

#endif MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_PLANNING_ARTICULATION_H_
