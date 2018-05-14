

#ifndef MODULES_CONTROL_CONTROLLER_TRUCKMODEL_SIX_AXLES_H_
#define MODULES_CONTROL_CONTROLLER_TRUCKMODEL_SIX_AXLES_H_

#include <fstream>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "modules/common/configs/proto/vehicle_config.pb.h"

// #include "modules/common/filters/digital_filter.h"
// #include "modules/common/filters/digital_filter_coefficients.h"
// #include "modules/common/filters/mean_filter.h"
// #include "modules/control/common/interpolation_1d.h"
// #include "modules/control/common/interpolation_2d.h"
#include "modules/control/common/trajectory_analyzer.h"
#include "modules/control/controller/controller.h"

namespace apollo{
namespace control{
// namespace math{

using Matrix = Eigen::MatrixXd;

class TruckModel6Axles
{
    public:
    TruckModel6Axles();
    ~TruckModel6Axles();

    void GetTruckModelMatrix(const Matrix &matrix_ad, const Matrix &matrix_cd,
                             const Matrix &matrix_d1d, const Matrix &matrix_d2d);
    void Init();
    // Matrix GetMatrixAD();
    // Matrix GetMatrixCD();
    // Matrix GetMatrixD1D();
    // Matrix GetMatrixD2D();
    
    protected:
    bool LoadModelConf(const ControlConf *control_conf);
    common::Status InitModelMatrixs(const ControlConf *control_conf);
    void UpdateModelMatrix(SimpleMPCDebug *debug);   

    double truck_mass_ = 0.0;
    double trailer_mass_ = 0.0;
    double para_C1_ = 0.0;
    double para_C2_ = 0.0;
    double para_C3_ = 0.0;
    double para_C4_ = 0.0;
    double para_C5_ = 0.0;
    double para_C6_ = 0.0;

    double para_b1_ = 0.0;
    double para_b2_ = 0.0;
    double para_b3_ = 0.0;
    double para_b4_ = 0.0;
    double para_b5_ = 0.0;
    double para_b6_ = 0.0;

    double para_hR_ = 0.0;
    double para_hF_ = 0.0;
    double para_I1_ = 0.0;
    double para_I2_ = 0.0;
    double sedan_Vx_ = 0.0;
    double ts_ = 0.0;
    int num_of_states_ = 8;

    // vehicle state matrix
    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_a_coeffi;
    // vehicle state matrix (discrete-time)
    // Eigen::MatrixXd matrix_ad_;

    // control matrix
    Eigen::MatrixXd matrix_c_;
    // control matrix (discrete-time)
    // Eigen::MatrixXd matrix_cd_;

    // offset matrix 1
    Eigen::MatrixXd matrix_d1_;
    Eigen::MatrixXd matrix_d1_coeffi;
    // offset matrix (discrete-time)
    // Eigen::MatrixXd matrix_d1d_;

    // offset matrix 2
    Eigen::MatrixXd matrix_d2_;
    Eigen::MatrixXd matrix_d2_coeffi;
    // offset matrix (discrete-time)
    // Eigen::MatrixXd matrix_d2d_;

    Eigen::MatrixXd matrix_m_;

    // Eigen::MatrixXd matrix_lm_;

    // Eigen::MatrixXd matrix_la_;

    // Eigen::MatrixXd matrix_lc_;

    // Eigen::MatrixXd matrix_m_;
    Eigen::MatrixXd matrix_ad_;
    Eigen::MatrixXd matrix_cd_;
    Eigen::MatrixXd matrix_d1d_;
    Eigen::MatrixXd matrix_d2d_;


};

// }// end namespace math
}//end namespace  control
}//end namespace apollo


#endif
