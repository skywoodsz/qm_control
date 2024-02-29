//
// Created by skywoodsz on 2023/4/9.
//

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

#include "qm_wbc/WbcBase.h"

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace qm{
using namespace ocs2;

WbcBase::WbcBase(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                 const PinocchioEndEffectorKinematics& armEeKinematics,
                 ros::NodeHandle &controller_nh)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      eeKinematics_(eeKinematics.clone()),
      armEeKinematics_(armEeKinematics.clone())
{
    numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts; // \dot v, f
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    qDesired_ = vector_t(info_.generalizedCoordinatesNum);
    vDesired_ = vector_t(info_.generalizedCoordinatesNum);
    inputLast_ = vector_t::Zero(info_.inputDim);
    baseAccDesired_ = vector_t(6);

    const auto& model = pinocchioInterfaceMeasured_.getModel();
    armEeFrameIdx_ = model.getBodyId(armEeKinematics_->getIds()[0]);

    base_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    base_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);

    arm_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);

    jointKp_ = matrix_t::Zero(6, 6);
    jointKd_ = matrix_t::Zero(6, 6);

    armEeLinearKp_ = matrix_t::Zero(3, 3);
    armEeLinearKd_ = matrix_t::Zero(3, 3);
    armEeAngularKp_ = matrix_t::Zero(3, 3);
    armEeAngularKd_ = matrix_t::Zero(3, 3);

    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh, "wbc");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}

void WbcBase::dynamicCallback(qm_wbc::WbcWeightConfig &config, uint32_t) {
    // joint
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

    // ee linear
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

    // base
    swingKp_ = config.kp_swing;
    swingKd_ = config.kd_swing;

    baseHeightKp_ = config.baseHeightKp;
    baseHeightKd_ = config.baseHeightKd;

    baseAngularKp_ = config.kp_base_angular;
    baseAngularKd_ = config.kd_base_angular;

    baseLinearKp_ = config.kp_base_linear;
    baseLinearKd_ = config.kd_base_linear;

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

    baseAccDesired_.setZero();
    baseAccDesired_ = AbInv * centroidalMomentumRate;
}

Task WbcBase::formulateBaseLinearMotionTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.head(2) + baseLinearKp_ * (qDesired_.head(2) - qMeasured_.head(2))
        + baseLinearKd_ * (vDesired_.head(2) - vMeasured_.head(2));

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base xy linear motion task
Task WbcBase::formulateBaseXYLinearAccelTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.segment<2>(0);

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base angular motion task
Task WbcBase::formulateBaseAngularMotionTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    vector3_t eulerAngles = qMeasured_.segment<3>(3);

    // from derivative euler to angular
    vector3_t vMeasuredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3));

    // from euler to rotation
    vector3_t eulerAnglesDesired;
    eulerAnglesDesired << qDesired_.segment<3>(3);
    matrix3_t rotationBaseMeasuredToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

    // desired acc
    vector3_t accDesired = getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles,
                                                                   vDesired_.segment<3>(3), baseAccDesired_.segment<3>(3));

    b = accDesired + baseAngularKp_  * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal)
                 - base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base height motion task
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

// [J, 0] x = \dot V - \dotJ v
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

// EoM
// [Mb, -J^Tb]x = -hb
Task WbcBase::formulateFloatingBaseEomTask() {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mb, Jb_T;
    vector_t hb;
    Mb = data.M.topRows(6);
    hb = data.nle.topRows(6);
    Jb_T = j_.transpose().topRows(6);

    a << Mb, -Jb_T;
    b = -hb;

    return {a, b, matrix_t(), matrix_t()};
}

// torque limit
// tau_min - hj <= [Mj, -Jj^T] <= tau_max - hj
Task WbcBase::formulateTorqueLimitsTask() {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);

    d.block(0, 0, info_.actuatedDofNum, numDecisionVars_) << Mj, -Jj_T;
    d.block(info_.actuatedDofNum, 0, info_.actuatedDofNum, numDecisionVars_) << -Mj, Jj_T;

    f << legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, armTorqueLimits_,
            legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, legTorqueLimits_, armTorqueLimits_;
    f.segment<18>(0) -= hj;
    f.segment<18>(info_.actuatedDofNum) += hj;

    return {matrix_t(), vector_t(), d, f};
}

// [J, 0] x = -\dot J v
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

// no contact:
// [0, I] x = 0
// contact:
// [0, C] x <= 0
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

    matrix_t frictionPyramic(5, 3);
    frictionPyramic << 0, 0, -1,
            1, 0, -frictionCoeff_,
            -1, 0, -frictionCoeff_,
            0, 1, -frictionCoeff_,
            0,-1, -frictionCoeff_;

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

Task WbcBase::formulateArmJointNomalTrackingTask()
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

    for (int i = 0; i < 6; ++i) {
        jointKp(i, i) = jointKp_(i, i);
        jointKd(i, i) = jointKd_(i, i);
    }

    b = jointKp * (qDesired_.segment<6>(info_.generalizedCoordinatesNum-6)
            - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6))
        + jointKd * (vDesired_.segment<6>(info_.generalizedCoordinatesNum-6)
                - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6));

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

    vector3_t linear_acc = armEeLinearKp_ * (posDesired[0] - posMeasured[0])
            + armEeLinearKd_ * (velDesired[0] - velMeasured[0]);

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) =
            arm_j_.block(0, 0, 3, info_.generalizedCoordinatesNum);

    b = linear_acc - arm_dj_.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateEeAngularMotionTrackingTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // measure
    const auto& Mmodel = pinocchioInterfaceMeasured_.getModel();
    auto& Mdata = pinocchioInterfaceMeasured_.getData();

    matrix3_t rotationEeMeasuredToWorld = Mdata.oMf[armEeFrameIdx_].rotation();
    vector3_t armCurrentEeAngularVel = pinocchio::getFrameVelocity(Mmodel, Mdata, armEeFrameIdx_,
                                                                   pinocchio::LOCAL_WORLD_ALIGNED).angular();

    // desired
    const auto& Dmodel = pinocchioInterfaceDesired_.getModel();
    auto& Ddata = pinocchioInterfaceDesired_.getData();
    matrix3_t rotationEeReferenceToWorld = Ddata.oMf[armEeFrameIdx_].rotation();
    vector3_t armDesiredEeAngularVel = pinocchio::getFrameVelocity(Dmodel, Ddata, armEeFrameIdx_,
                                                   pinocchio::LOCAL_WORLD_ALIGNED).angular();

    // error
    vector3_t error = rotationErrorInWorld<scalar_t>(rotationEeReferenceToWorld, rotationEeMeasuredToWorld);

    matrix_t arm_dj_tmp = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_tmp.setZero();
    arm_dj_tmp = arm_dj_;
    arm_dj_tmp.block(3, 3, 3, 3).setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) =
            arm_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);
    a.block(0, 3, 3, 3).setZero();

    b = armEeAngularKp_ * error + armEeAngularKd_ * (- armCurrentEeAngularVel)
        - arm_dj_tmp.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

// [0, I] x = GRFs
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

vector_t WbcBase::updateCmd(ocs2::vector_t x_optimal) {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);
    matrix_t a = (matrix_t(info_.actuatedDofNum, getNumDecisionVars())<< Mj, -Jj_T).finished();

    vector_t torque_optimal = a * x_optimal + hj;

    vector_t cmd = (vector_t(numDecisionVars_ + info_.actuatedDofNum)<<x_optimal, torque_optimal).finished();

    return cmd;
}

void WbcBase::loadTasksSetting(const std::string &taskFile, bool verbose) {
    // Load task file
    legTorqueLimits_ = vector_t(3);
    legTorqueLimits_ = pinocchioInterfaceMeasured_.getModel().effortLimit.segment<3>(6);

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

}

}

