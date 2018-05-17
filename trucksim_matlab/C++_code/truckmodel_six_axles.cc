
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
// #include "modules/common/math/mpc_solver.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "Eigen/LU"
#include "modules/control/controller/truckmodel_six_axles.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

namespace math = apollo::common::math;

namespace apollo{
namespace control{
// namespace math{ 


using apollo::common::ErrorCode;
using apollo::common::Point3D;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using Matrix = Eigen::MatrixXd;
using apollo::common::VehicleConfigHelper;

TruckModel6Axles::TruckModel6Axles():num_of_states_(8){};

TruckModel6Axles::~TruckModel6Axles(){};

bool TruckModel6Axles::LoadModelConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR << "[MPCController] control_conf == nullptr";
    return false;
  }
  // const auto &vehicle_param =
  //     VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  // CHECK_GT(ts_, 0.0) << "[MPCController] Invalid control update interval.";
//   para_C1_ = control_conf->mpc_controller_conf().C1();
//   para_C2_ = control_conf->mpc_controller_conf().C2();
//   para_C3_ = control_conf->mpc_controller_conf().C3();
//   para_C4_ = control_conf->mpc_controller_conf().C4();
//   para_C5_ = control_conf->mpc_controller_conf().C5();
//   para_C6_ = control_conf->mpc_controller_conf().C6();
  para_C1_ = control_conf->mpc_controller_conf().cf();
  para_C2_ = control_conf->mpc_controller_conf().cr();
  para_C3_ = control_conf->mpc_controller_conf().cf();
  para_C4_ = control_conf->mpc_controller_conf().cr();
  para_C5_ = control_conf->mpc_controller_conf().cf();
  para_C6_ = control_conf->mpc_controller_conf().cr();
  // wheelbase_ = vehicle_param.wheel_base();

  // max_lat_acc_ = control_conf->mpc_controller_conf().max_lateral_acceleration();

//   truck_mass_ = control_conf->mpc_controller_conf().truck_mass();
//   trailer_mass_ = mass_rr = control_conf->mpc_controller_conf().trailer_mass_();
  truck_mass_ = control_conf->mpc_controller_conf().mass_fl();
  trailer_mass_ = control_conf->mpc_controller_conf().mass_rr();

//   para_b1_ = control_conf->mpc_controller_conf().para_b1();
//   para_b2_ = control_conf->mpc_controller_conf().para_b2();
//   para_b3_ = control_conf->mpc_controller_conf().para_b3();
//   para_b4_ = control_conf->mpc_controller_conf().para_b4();
//   para_b5_ = control_conf->mpc_controller_conf().para_b5();
//   para_b6_ = control_conf->mpc_controller_conf().para_b6();
  para_b1_ = 1.0;
  para_b2_ = 1.0;
  para_b3_ = 1.0;
  para_b4_ = 1.0;
  para_b5_ = 1.0;
  para_b6_ = 1.0;

//   para_hF_ = control_conf->mpc_controller_conf().para_hF();
//   para_hR_ = control_conf->mpc_controller_conf().para_hF();
  para_hF_ = control_conf->mpc_controller_conf().mass_rr();
  para_hR_ = control_conf->mpc_controller_conf().mass_rr();

//   iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  const double truck_mass_f = truck_mass_ * 0.75;
  const double truck_mass_r = truck_mass_ * 0.25;
  const double trailer_mass_f = trailer_mass_ * 0.25;
  const double trailer_mass_r = trailer_mass_ * 0.75;
  para_I1_ = truck_mass_f * para_b1_ * para_b1_ + truck_mass_r * para_hF_ * para_hF_;
  para_I2_ = trailer_mass_f * para_hR_ * para_hR_ + trailer_mass_r * para_b5_ * para_b5_;

//   LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  AINFO << "truckmodel conf loaded";
  return true;
}

Status TruckModel6Axles::InitModelMatrixs(const ControlConf *control_conf)
{
  if (!LoadModelConf(control_conf)) {
    AERROR << "failed to load model conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load model_conf");
  }

  matrix_m_ = Matrix::Zero(num_of_states_, num_of_states_);
  matrix_m_(0,0) = 1.0;
  matrix_m_(1,1) = truck_mass_;
  matrix_m_(1,5) = trailer_mass_;
  matrix_m_(2,2) = 1.0;
  matrix_m_(3,3) = para_I1_;
  matrix_m_(3,5) = -para_hF_ * trailer_mass_;
  matrix_m_(4,4) = 1.0;
  matrix_m_(5,5) = -para_hR_ * trailer_mass_;
  matrix_m_(5,7) = -para_I2_;
  matrix_m_(6,6) = 1.0;
  matrix_m_(7,3) = -para_hF_ ;
  matrix_m_(7,5) = -1.0;
  matrix_m_(7,7) = -para_hR_ ;

  matrix_a_ = Matrix::Zero(num_of_states_, num_of_states_);
  matrix_ad_ = Matrix::Zero(num_of_states_, num_of_states_);
  matrix_a_coeffi = Matrix::Zero(num_of_states_, num_of_states_);
  matrix_a_(0,1) = 1.0;
  matrix_a_(1,1) = (-para_C1_ - para_C2_ - para_C3_);// /Vx
  matrix_a_(1,2) = (para_C1_ + para_C2_ + para_C3_);
  matrix_a_(1,3) = (-para_b1_ * para_C1_ + para_b2_ * para_C2_ + para_C3_ * para_b3_ );// /Vx
  matrix_a_(1,5) = (-para_C4_ - para_C5_ - para_C6_);// /Vx
  matrix_a_(1,6) = (para_C4_ + para_C5_ + para_C6_);
  matrix_a_(1,7) = (para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_C6_ * para_b6_ );// /Vx
  matrix_a_(2,3) = 1.0;
  matrix_a_(3,1) = (-para_b1_ * para_C1_ + para_b2_ * para_C2_ + para_C3_ * para_b3_ );// /Vx
  matrix_a_(3,2) = (para_b1_ * para_C1_ - para_b2_ * para_C2_ - para_C3_ * para_b3_ );
  matrix_a_(3,3) = ( -para_C1_ * para_b1_ * para_b1_ -para_C2_ * para_b2_ * para_b2_ - para_C3_ * para_b3_ * para_b3_); // /Vx
  matrix_a_(3,5) = ( para_hF_ * para_C4_ + para_hF_ * para_C5_ + para_hF_ * para_C6_ );// /Vx
  matrix_a_(3,6) = -( para_hF_ * para_C4_ + para_hF_ * para_C5_ + para_hF_ * para_C6_ );
  matrix_a_(3,7) = -para_hF_ * (para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_C6_ * para_b6_ );
  matrix_a_(4,5) = 1.0;
  matrix_a_(5,5) = (para_hR_ * ( para_C4_ + para_C5_ + para_C6_ ) + para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_ );// /Vx
  matrix_a_(5,6) = -para_hR_ * ( para_C4_ + para_C5_ + para_C6_ ) -( para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_) ;
  matrix_a_(5,7) = (-para_hR_ * ( para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_ ) - 
                    ( para_C4_ * para_b4_ * para_b4_ + para_C5_ * para_b5_ * para_b5_ + para_C6_ * para_b6_ * para_b6_));// /Vx
  matrix_a_(6,7) = 1.0;



  matrix_c_ = Matrix::Zero(num_of_states_, 1);
  matrix_c_(1,0) = para_C1_;
  matrix_c_(3,0) = para_b1_ * para_C1_;

  matrix_d1_ = Matrix::Zero(num_of_states_, 1);
  matrix_d1d_ = Matrix::Zero(num_of_states_, 1);
  // matrix_d1_(3,0)
  matrix_d2_ = Matrix::Zero(num_of_states_, 1);
  matrix_d2d_ = Matrix::Zero(num_of_states_, 1);

  return Status::OK();
}

void TruckModel6Axles::UpdateModelMatrix(SimpleMPCDebug *debug)
{
  const double v = VehicleStateProvider::instance()->linear_velocity();
  // matrix_a_(1,1) = (-para_C1_ - para_C2_ - para_C3_) / v;
  // matrix_a_(1,3) = (-para_b1_ * para_C1_ + para_b2_ * para_C2_ + para_C3_ * para_b3_ ) / v;
  // matrix_a_(1,5) = (-para_C4_ - para_C5_ - para_C6_) / v;
  // matrix_a_(1,7) = (para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_C6_ * para_b6_ ) / v;
  // matrix_a_(3,1) = (-para_b1_ * para_C1_ + para_b2_ * para_C2_ + para_C3_ * para_b3_ ) / v;
  // matrix_a_(3,3) = ( -para_C1_ * para_b1_ * para_b1_ -para_C2_ * para_b2_ * para_b2_ - para_C3_ * para_b3_ * para_b3_) / v;
  // matrix_a_(3,5) = ( para_hF_ * para_C4_ + para_hF_ * para_C5_ + para_hF_ * para_C6_ ) / v;
  // matrix_a_(5,5) = (para_hR_ * ( para_C4_ + para_C5_ + para_C6_ ) + para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_ ) / v;
  // matrix_a_(5,7) = (-para_hR_ * ( para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_ ) - 
  //                   ( para_C4_ * para_b4_ * para_b4_ + para_C5_ * para_b5_ * para_b5_ + para_C6_ * para_b6_ * para_b6_)) / v;
  matrix_a_(1,1) = matrix_a_(1,1) / v;
  matrix_a_(1,3) = matrix_a_(1,3) / v;
  matrix_a_(1,5) = matrix_a_(1,5) / v;
  matrix_a_(1,7) = matrix_a_(1,7) / v;
  matrix_a_(3,1) = matrix_a_(3,1) / v;
  matrix_a_(3,3) = matrix_a_(3,3) / v;
  matrix_a_(3,5) = matrix_a_(3,5) / v;
  matrix_a_(5,5) = matrix_a_(5,5) / v;
  matrix_a_(5,7) = matrix_a_(5,7) / v;

  matrix_d1_(1,0) = ( -para_b1_ * para_C1_ + para_b2_ * para_C2_ + para_b3_ * para_C3_) / v - trailer_mass_ * v;
  matrix_d1_(3,0) = ( -para_C1_ * para_b1_ * para_b1_ - para_C2_ * para_b2_ * para_b2_ - para_C3_ * para_b3_ * para_b3_ ) / v;
  matrix_d1_(7,0) = -v;

  matrix_d2_(1,0) = ( para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_) / v - trailer_mass_ * v;
  matrix_d2_(3,0) = -para_hF_ *(  para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_) / v + para_hF_ * trailer_mass_ * v;
  matrix_d1_(7,0) = ( -para_hR_ * (para_b4_ * para_C4_ + para_b5_ * para_C5_ + para_b6_ * para_C6_) - 
                     (para_C4_ * para_b4_ * para_b4_ + para_C5_ * para_b5_ * para_b5_ + para_C6_ * para_b6_ * para_b6_) ) / v 
                     + para_hR_ * trailer_mass_ * v;
  matrix_d1_(7,0) = -v;

  // Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  // matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
  //              (matrix_i + ts_ * 0.5 * matrix_a_);

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_m_.inverse() * matrix_a_ ) * matrix_i.inverse() -
                ( ts_ * 0.5 * matrix_m_.inverse() * matrix_a_);
  
  matrix_cd_ = matrix_m_.inverse() * matrix_c_ * ts_ ;
  // double truck_angular_v = debug ->set_speed_reference() * debug ->set_curvature();
  matrix_d1d_ = matrix_m_.inverse() * matrix_d1_ * ts_ * debug->heading_error_rate();//heading_error_rate
  // double trailer_angular_v = debug ->set_speed_reference() * debug ->set_curvature();
  // matrix_d2d_ = matrix_m_.inverse() * matrix_d2_ * ts_ * debug ->trailer_heading_error_rate();
  matrix_d2d_ = matrix_m_.inverse() * matrix_d2_ * ts_ * debug ->heading_error_rate();


}








 
// }//end namespace math
}//end namespace control
}//end namespace apollo
