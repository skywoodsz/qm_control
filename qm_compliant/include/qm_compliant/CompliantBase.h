//
// Created by skywoodsz on 2023/4/22.
//

#ifndef SRC_COMPLIANTBASE_H
#define SRC_COMPLIANTBASE_H

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
using vector6_t = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;

namespace qm{
using namespace ocs2;
class CompliantBase{

public:
    CompliantBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                  const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    virtual vector_t update(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);

    size_t getGeneralizedCoordinatesNum(){ return info_.generalizedCoordinatesNum; };
    vector3_t getOrientationError(const vector3_t& eularDesired);

protected:
    vector_t qMeasured_, vMeasured_;
    matrix_t arm_j_, arm_dj_;

    vector3_t x_, v_, w_;
    vector_t g_;

    size_t generalizedCoordinatesNum_;
    size_t actuatedDofNum_;

private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;

    std::unique_ptr<PinocchioEndEffectorKinematics> armEeKinematics_;

    matrix3_t rotationEeMeasuredToWorld_;
};
}



#endif //SRC_COMPLIANTBASE_H
