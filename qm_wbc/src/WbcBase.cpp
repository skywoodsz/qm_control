//
// Created by skywoodsz on 2023/3/12.
//

#include <pinocchio/fwd.hpp>

#include "qm_wbc/WbcBase.h"
#include <qm_common/math_tool/mathConvert.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace qm{

WbcBase::WbcBase(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                 const PinocchioEndEffectorKinematics& armEeKinematics,
                 ros::NodeHandle &controller_nh)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()),
      armEeKinematics_(armEeKinematics.clone()){
    numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    qDesired_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vDesired_ = vector_t(info_.generalizedCoordinatesNum);
    base_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    base_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
    baseAccDesired_ = vector_t(6);

    arm_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);

    armEeLinearKp_ = matrix_t::Zero(3, 3);
    armEeLinearKd_ = matrix_t::Zero(3, 3);

    jointKp_ = matrix_t::Zero(6, 6);
    jointKd_ = matrix_t::Zero(6, 6);

    armEeAngularKp_ = matrix_t::Zero(3, 3);
    armEeAngularKd_ = matrix_t::Zero(3, 3);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    armEeFrameIdx_ = model.getBodyId(armEeKinematics_->getIds()[0]);

    zyx2xyz_.setZero();
    zyx2xyz_ << 0., 0., 1., 0., 1., 0., 1., 0., 0.;

    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh, "wbc");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}

void WbcBase::dynamicCallback(qm_wbc::WbcWeightConfig &config, uint32_t) {
    baseHeightKp_ = config.baseHeightKp;
    baseHeightKd_ = config.baseHeightKd;

    baseLinearKp_ = config.kp_base_linear;
    baseLinearKd_ = config.kd_base_linear;
    baseAngularKp_ = config.kp_base_angular;
    baseAngularKd_ = config.kd_base_angular;

    jointKp_(0, 0) = config.kp_arm_joint_1;
    jointKp_(1, 1) = config.kp_arm_joint_2;
    jointKp_(2, 2) = config.kp_arm_joint_3;
    jointKp_(3, 3) = config.kp_arm_joint_4;
    jointKp_(4, 4) = config.kp_arm_joint_5;
    jointKp_(5, 5) = config.kp_arm_joint_6;

    jointKd_(0, 0) = config.kd_arm_joint_1;
    jointKd_(1, 1) = config.kd_arm_joint_2;
    jointKd_(2, 2) = config.kd_arm_joint_3;
    jointKd_(3, 3) = config.kd_arm_joint_4;
    jointKd_(4, 4) = config.kd_arm_joint_5;
    jointKd_(5, 5) = config.kd_arm_joint_6;

    armEeLinearKp_(0, 0) = config.kp_ee_linear_x;
    armEeLinearKp_(1, 1) = config.kp_ee_linear_y;
    armEeLinearKp_(2, 2) = config.kp_ee_linear_z;

    armEeLinearKd_(0, 0) = config.kd_ee_linear_x;
    armEeLinearKd_(1, 1) = config.kd_ee_linear_y;
    armEeLinearKd_(2, 2) = config.kd_ee_linear_z;

    armEeAngularKp_(0, 0) = config.kp_ee_angular_x;
    armEeAngularKp_(1, 1) = config.kp_ee_angular_y;
    armEeAngularKp_(2, 2) = config.kp_ee_angular_z;

    armEeAngularKd_(0, 0) = config.kd_ee_angular_x;
    armEeAngularKd_(1, 1) = config.kd_ee_angular_y;
    armEeAngularKd_(2, 2) = config.kd_ee_angular_z;

    kt_ = config.kt;

    ROS_INFO_STREAM("\033[32m Update the wbc param. \033[0m");
}

vector_t WbcBase::update(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                         const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period, scalar_t time) {
    contactFlag_ = modeNumber2StanceLeg(mode);
    numContacts_ = 0;
    for (bool flag : contactFlag_) {
        if (flag) {
            numContacts_++;
        }
    }

    updateMeasured(rbdStateMeasured);
    updateDesired(stateDesired, inputDesired, period);
    return {};
}


void WbcBase::updateMeasured(const ocs2::vector_t &rbdStateMeasured) {

    qMeasured_.setZero();
    vMeasured_.setZero();

    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    auto& data = pinocchioInterfaceMeasured_.getData();

    // For floating base EoM task
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);

    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // For floating base EoM task
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // For not contact motion task
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // For base motion tracking task
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> base_j, base_dj;
    base_j.setZero(6, info_.generalizedCoordinatesNum);
    base_dj.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j);
    pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj);
    base_j_.setZero(); base_j_ = base_j;
    base_dj_.setZero(); base_dj_ = base_dj;

    // For armEE motion tracking
    arm_j_.setZero();
    arm_dj_.setZero(); // must
    pinocchio::getFrameJacobian(model, data, armEeFrameIdx_, pinocchio::LOCAL_WORLD_ALIGNED, arm_j_);
    pinocchio::getFrameJacobianTimeVariation(model, data, armEeFrameIdx_, pinocchio::LOCAL_WORLD_ALIGNED, arm_dj_);

}

void WbcBase::updateDesired(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired, ocs2::scalar_t period) {
    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();

    qDesired_.setZero();
    vDesired_.setZero();

    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
    qDesired_ = mapping_.getPinocchioJointPosition(stateDesired);
    pinocchio::forwardKinematics(model, data, qDesired_);
    pinocchio::computeJointJacobians(model, data, qDesired_);
    pinocchio::updateFramePlacements(model, data);
    updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired_);

    vDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);

    // update base acc desired
    jointAccel_ = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;

    const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    auto Aj = A.rightCols(info_.actuatedDofNum);
    const auto ADot = pinocchio::dccrba(model, data, qDesired_, vDesired_);
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired_;
    centroidalMomentumRate.noalias() -= Aj * jointAccel_;

    baseAccDesired_ = AbInv * centroidalMomentumRate;
}

Task WbcBase::formulateEeAngularMotionTrackingTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // current
    const auto& Mmodel = pinocchioInterfaceMeasured_.getModel();
    auto& Mdata = pinocchioInterfaceMeasured_.getData();
    const auto armCurrentEeOriQuat = matrixToQuaternion(Mdata.oMf[armEeFrameIdx_].rotation());
    const auto armCurrentEeAngularVel = pinocchio::getFrameVelocity(Mmodel, Mdata, armEeFrameIdx_,
                                                                    pinocchio::LOCAL_WORLD_ALIGNED).angular();
    auto armCurrentEeOriEular = quatToZyx(armCurrentEeOriQuat);
    makeEulerAnglesUnique<scalar_t>(armCurrentEeOriEular);

    // desired
    const auto& Dmodel = pinocchioInterfaceDesired_.getModel();
    auto& Ddata = pinocchioInterfaceDesired_.getData();
    const auto armDesiredEeOriQuat = matrixToQuaternion(Ddata.oMf[armEeFrameIdx_].rotation());
    const auto armDesiredEeAngularVel = pinocchio::getFrameVelocity(Dmodel, Ddata, armEeFrameIdx_,
                                                                    pinocchio::LOCAL_WORLD_ALIGNED).angular();
    auto armDesiredEeOriEular = quatToZyx(armDesiredEeOriQuat);
    makeEulerAnglesUnique<scalar_t>(armDesiredEeOriEular);

    // desired acc
    vector3_t angular_acc = armEeAngularKp_ * zyx2xyz_ * (armDesiredEeOriEular - armCurrentEeOriEular)
                            + armEeAngularKd_ * (- armCurrentEeAngularVel);


    a.block(0, info_.generalizedCoordinatesNum-6, 3, 6) =
            arm_j_.block(3, info_.generalizedCoordinatesNum-6, 3, 6);
    b = angular_acc - arm_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateEeLinearMotionTrackingTask() {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // current
    armEeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = armEeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = armEeKinematics_->getVelocity(vector_t(), vector_t());

    // desired
    armEeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = armEeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = armEeKinematics_->getVelocity(vector_t(), vector_t());

    vector3_t linear_acc = armEeLinearKp_ * (posDesired[0] - posMeasured[0]) + armEeLinearKd_ * (velDesired[0] - velMeasured[0]);

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = arm_j_.block(0, 0, 3, info_.generalizedCoordinatesNum);
    b = linear_acc - arm_dj_.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

/**
 * arm joint vel limit
 */
Task WbcBase::formulateManipulatorJointVelLimit(scalar_t period) {
    matrix_t d(12, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    matrix_t i = matrix_t::Identity(6, 6);
    scalar_t dealta_t = kt_ * period;

    d.block(0, info_.generalizedCoordinatesNum-6, 6, 6) = i * dealta_t;
    d.block(6, info_.generalizedCoordinatesNum-6, 6, 6) = - i * dealta_t;

    f.segment<6>(0) = armJointVelLimits_.head(6) - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6); // upper limit
    f.segment<6>(6) = - armJointVelLimits_.tail(6) + vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6); // lower limit

    return {matrix_t{}, vector_t{}, d, f};
}

/**
 * arm joint pos limit
 */
Task WbcBase::formulateManipulatorJointPosLimit(ocs2::scalar_t period) {
    matrix_t d(12, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    d.block(0, info_.generalizedCoordinatesNum-6, 6, 6) = 0.5 * period * period * matrix_t::Identity(6, 6);
    d.block(6, info_.generalizedCoordinatesNum-6, 6, 6) = - 0.5 * period * period * matrix_t::Identity(6, 6);

    f.segment<6>(0) = armJointPosLimits_.head(6)
            - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6)
             - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6) * period;

    f.segment<6>(6) = - armJointPosLimits_.tail(6)
                      + qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6)
                      + vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6) * period;

    return {matrix_t{}, vector_t{}, d, f};
}

Task WbcBase::formulateArmJointTrackingTask()
{
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, info_.generalizedCoordinatesNum-6, 6, 6) = matrix_t::Identity(6, 6);

    matrix_t jointKp, jointKd;
    vector_t jointDesPos;
    jointKp = matrix_t::Zero(6, 6);
    jointKd = matrix_t::Zero(6, 6);
    jointDesPos = vector_t::Zero(6, 6);

    for (int i = 0; i < 6; ++i) {
        jointKp(i, i) = jointKp_(i, i);
        jointKd(i, i) = jointKd_(i, i);
    }

    b = jointKp * (qDesired_.segment<6>(info_.generalizedCoordinatesNum-6) - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6))
        + jointKd * (vDesired_.segment<6>(info_.generalizedCoordinatesNum-6) - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6)); //

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateArmJointNomalTrackingTask() {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, info_.generalizedCoordinatesNum-6, 6, 6) = matrix_t::Identity(6, 6);

    matrix_t jointKp, jointKd;
    vector_t jointDesPos;
    jointKp = matrix_t::Zero(6, 6);
    jointKd = matrix_t::Zero(6, 6);
    jointDesPos = vector_t::Zero(6, 6);

    for (int i = 0; i < 6; ++i) {
        jointKp(i, i) = jointKp_(i, i);
        jointKd(i, i) = jointKd_(i, i);
    }

    b = jointKp * (jointDesPos - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6))
        - jointKd * vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6);

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateBaseXYAngularMotionTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 4, 2, 2) = base_j_.block(3, 4, 2, 2);

    vector_t vMeasuredGlobal(vMeasured_.rows());
    vector_t vDesiredGlobal(vMeasured_.rows());
    vMeasuredGlobal = vMeasured_;
    vMeasuredGlobal.segment<3>(3) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(qMeasured_.segment<3>(3),
                                                                                                    vMeasured_.segment<3>(3));
    vDesiredGlobal = vDesired_;
    vDesiredGlobal.segment<3>(3) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(qDesired_.segment<3>(3),
                                                                                                   vDesired_.segment<3>(3));
    const matrix_t yx2xy = (matrix_t(2, 2) << 0.0, 1.0, 1.0, 0.0).finished();

    b = baseAngularKd_ * (vDesiredGlobal.segment<2>(3) - vMeasuredGlobal.segment<2>(3))
        + baseAngularKp_ * yx2xy * (qDesired_.segment<2>(4) - qMeasured_.segment<2>(4))
        - base_dj_.block(4, 0, 2, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateBaseZAngularAccelTask()
{
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 3, 1, 1) = matrix_t::Identity(1, 1);

    b = baseAccDesired_.segment<1>(3);

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateBaseXYLinearAccelTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.segment<2>(0);

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateBaseXYAngularAccelTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 4, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.segment<2>(4);

    return {a, b, matrix_t(), vector_t()};
}

/**
 * Tracking base angular motion task
 * TODO：add desired acc
 */
Task WbcBase::formulateBaseAngularMotion() {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 3, 3, 3) = base_j_.block(3, 3, 3, 3);

    vector_t vMeasuredGlobal(vMeasured_.rows());
    vector_t vDesiredGlobal(vMeasured_.rows());
    vMeasuredGlobal = vMeasured_;
    vMeasuredGlobal.segment<3>(3) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(qMeasured_.segment<3>(3),
                                                                                              vMeasured_.segment<3>(3));
    vDesiredGlobal = vDesired_;
    vDesiredGlobal.segment<3>(3) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(qDesired_.segment<3>(3),
                                                                                              vDesired_.segment<3>(3));

    b =  zyx2xyz_ * baseAccDesired_.segment<3>(3) +  baseAngularKd_ * (vDesiredGlobal.segment<3>(3) - vMeasuredGlobal.segment<3>(3))
         + baseAngularKp_ * zyx2xyz_ * (qDesired_.segment<3>(3) - qMeasured_.segment<3>(3))
         - base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

/**
 * Tracking base linear xy motion task
 * TODO：add desired acc
 */
Task WbcBase::formulateBaseXYLinearMotionTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);


    b = baseLinearKp_ * (qDesired_.head(2) - qMeasured_.head(2))
        + baseLinearKd_ * (vDesired_.head(2)- vMeasured_.head(2));

    return {a, b, matrix_t(), vector_t()};
}

/**
 * Tracking base linear motion task
 * TODO：add desired acc
 */
Task WbcBase::formulateBaseLinearMotionTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, 3) = matrix_t::Identity(3, 3);


    b = baseLinearKp_ * (qDesired_.head(3) - qMeasured_.head(3))
        + baseLinearKd_ * (vDesired_.head(3) - vMeasured_.head(3));

    return {a, b, matrix_t(), vector_t()};
}

/**
 * Tracking base height motion task
 */
Task WbcBase::formulateBaseHeightMotionTask() {
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 2, 1, 1) = matrix_t::Identity(1, 1);

    b[0] = baseAccDesired_[2] + baseHeightKp_ * (qDesired_[2] - qMeasured_[2])
           + baseHeightKd_ * (vDesired_[2] - vMeasured_[2]);

    return {a, b, matrix_t(), vector_t()};
}


// [M, -J^T, -S^T] x = -h
Task WbcBase::formulateFloatingBaseEomTask() {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_)
            << data.M, -j_.transpose(), -s.transpose()).finished();
    vector_t b = -data.nle;

    return {a, b, matrix_t(), matrix_t()};
}

// [J, 0, 0] x = -\dot J v
Task WbcBase::formulateNoContactMotionTask() {
    matrix_t a(3 * numContacts_, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
        if (contactFlag_[i]) {
            a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
            b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
            j++;
        }
    }

    return {a, b, matrix_t(), vector_t()};
}

// | 0,  I|    | \tau^max |
// |      | <= |          |
// | 0, -I|    | \tau^min |
Task WbcBase::formulateTorqueLimitsTask() {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    d.setZero();
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
            info_.actuatedDofNum, info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
            info_.actuatedDofNum, info_.actuatedDofNum) = -i;

    vector_t f(2 * info_.actuatedDofNum);
    f << legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, armTorqueLimits_,
    legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, armTorqueLimits_;

    return {matrix_t(), vector_t(), d, f};
}

// no contact:
// [0, I, 0] x = 0
// contact:
// [0, C, 0] x <= 0
Task WbcBase::formulateFrictionConeTask() {
    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (!contactFlag_[i]) {
            a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
    }
    vector_t b(a.rows());
    b.setZero();

    matrix_t frictionPyramic(5, 3);  // clang-format off
    frictionPyramic << 0, 0, -1,
            1, 0, -frictionCoeff_,
            -1, 0, -frictionCoeff_,
            0, 1, -frictionCoeff_,
            0,-1, -frictionCoeff_;  // clang-format on

    matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (contactFlag_[i]) {
            d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
        }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows());

    return {a, b, d, f};
}


Task WbcBase::formulateBaseAccelTask(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                                         ocs2::scalar_t period) {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    // desired acc
    vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;
    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    const auto Aj = A.rightCols(info_.actuatedDofNum);
    const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired;
    centroidalMomentumRate.noalias() -= Aj * jointAccel;

    Vector6 b = AbInv * centroidalMomentumRate;

    return {a, b, matrix_t(), vector_t()};
}

// [J, 0, 0] x = \dot V - \dotJ v
Task WbcBase::formulateSwingLegTask() {
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (!contactFlag_[i]) {
            vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
            a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
            b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
            j++;
        }
    }

    return {a, b, matrix_t(), vector_t()};
}

// [0, I, 0] x = GRFs
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
    matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
    b = inputDesired.head(a.rows());

    return {a, b, matrix_t(), vector_t()};
}


void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
    // Load task file
    legTorqueLimits_ = vector_t(3);
    loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", legTorqueLimits_);

    // arm torque limit
    armTorqueLimits_ = vector_t(6);
    armTorqueLimits_ = pinocchioInterfaceMeasured_.getModel().effortLimit.tail(6);

    if (verbose) {
        std::cerr << "\n #### Torque Limits Task:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### HAA HFE KFE: " << legTorqueLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
        std::cerr << "\n #### manipulator joint: " << armTorqueLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
    }

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "frictionConeTask.";
    if (verbose) {
        std::cerr << "\n #### Friction Cone Task:";
        std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
    if (verbose) {
        std::cerr << " #### =============================================================================\n";
    }
    prefix = "swingLegTask.";
    if (verbose) {
        std::cerr << "\n #### Swing Leg Task:";
        std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);

    armJointVelLimits_ = vector_t(12);
    vector_t lowerBoundArm = vector_t::Zero(6);
    vector_t upperBoundArm = vector_t::Zero(6);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);

    armJointVelLimits_.head(6) = upperBoundArm;
    armJointVelLimits_.tail(6) = lowerBoundArm;

    if (verbose) {
        std::cerr << "\n #### Manipulator's Joint Velocity Limits Task:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### manipulator joint: " << armJointVelLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
    }

    armJointPosLimits_ = vector_t(12);
    armJointPosLimits_.tail(6) = pinocchioInterfaceMeasured_.getModel().lowerPositionLimit.tail(6);
    armJointPosLimits_.head(6) = pinocchioInterfaceMeasured_.getModel().upperPositionLimit.tail(6);

    if (verbose) {
        std::cerr << "\n #### Manipulator's Joint Position Limits Task:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### manipulator joint: " << armJointPosLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
    }
}


}


