//
// Created by skywoodsz on 2023/4/22.
//

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include "qm_compliant/CompliantBase.h"

namespace qm{
using namespace ocs2;
CompliantBase::CompliantBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
     : pinocchioInterface_(pinocchioInterface),
       info_(std::move(info)),
       mapping_(info_),
       armEeKinematics_(armEeKinematics.clone())
{
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    arm_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
    g_ = vector_t(info_.generalizedCoordinatesNum);

    generalizedCoordinatesNum_ = info_.generalizedCoordinatesNum;
    actuatedDofNum_ = info_.actuatedDofNum;
}

vector_t CompliantBase::update(const vector_t &rbdStateMeasured, scalar_t time, scalar_t period) {
    qMeasured_.setZero();
    vMeasured_.setZero();

    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);

    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

    // g(q)
    pinocchio::computeGeneralizedGravity(model, data, qMeasured_);
    g_ = data.g;

    // operational jacobian
    arm_j_.setZero();
    arm_dj_.setZero(); // must
    size_t armEeFrameIdx = model.getBodyId(armEeKinematics_->getIds()[0]);
    pinocchio::getFrameJacobian(model, data, armEeFrameIdx, pinocchio::LOCAL_WORLD_ALIGNED, arm_j_);
    pinocchio::getFrameJacobianTimeVariation(model, data, armEeFrameIdx, pinocchio::LOCAL_WORLD_ALIGNED, arm_dj_);

    // operational position
    armEeKinematics_->setPinocchioInterface(pinocchioInterface_);
    x_ = armEeKinematics_->getPosition(vector_t()).front();
    v_ = armEeKinematics_->getVelocity(vector_t(), vector_t()).front();
    rotationEeMeasuredToWorld_ = data.oMf[armEeFrameIdx].rotation();
    w_.setZero();
    w_ = pinocchio::getFrameVelocity(model, data, armEeFrameIdx, pinocchio::LOCAL_WORLD_ALIGNED).angular();

    return {};
}

vector3_t CompliantBase::getOrientationError(const vector3_t &eularDesired) {

    matrix3_t rotationEeReferenceToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eularDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationEeReferenceToWorld, rotationEeMeasuredToWorld_);

    return error;
}

}
