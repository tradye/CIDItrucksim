/**
 * Copyright 2018 CiDi
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAILER_TRAJECTORY_GENERATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAILER_TRAJECTORY_GENERATOR_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class TrailerTrajectoryGenerator {
 public:
  static DiscretizedTrajectory Generate(const DiscretizedTrajectory
					truck_trajectory,
					Frame* frame);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAILER_TRAJECTORY_GENERATOR_H_
