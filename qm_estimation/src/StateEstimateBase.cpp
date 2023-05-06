//
// Created by skywoodsz on 2023/2/28.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "qm_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics,
                                     std::vector<HybridJointHandle> hybridJointHandles,
                                     std::vector<ContactSensorHandle> contactSensorHandles,
                                     hardware_interface::ImuSensorHandle imuSensorHandle)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()),
          armEeKinematics_(armEeKinematics.clone()),
          generalizedCoordinatesNum_(info_.generalizedCoordinatesNum),
          rbdState_(2 * generalizedCoordinatesNum_ + 7 + info_.actuatedDofNum + 3), // + ee pose + joint effort + force linear
          hybridJointHandles_(std::move(hybridJointHandles)),
          contactSensorHandles_(std::move(contactSensorHandles)),
          imuSensorHandle_(std::move(imuSensorHandle)) {
    ros::NodeHandle nh;
    odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));
    odomPub_->msg_.header.frame_id = "odom";
    odomPub_->msg_.child_frame_id = "base";

    posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));
    posePub_->msg_.header.frame_id = "odom";

    armEeKinematics_->setPinocchioInterface(pinocchioInterface_);
}

size_t StateEstimateBase::getMode() {
    contact_flag_t contactFlag;
    for (size_t i = 0; i < contactSensorHandles_.size(); ++i) {
        contactFlag[i] = contactSensorHandles_[i].isContact();
    }

    return stanceLeg2ModeNumber(contactFlag);
}

void StateEstimateBase::updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angularVel) {
    rbdState_.segment<3>(0) = quatToZyx(quat);
    rbdState_.segment<3>(generalizedCoordinatesNum_) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
    rbdState_.segment<3>(3) = pos;
    rbdState_.segment<3>(generalizedCoordinatesNum_ + 3) = linearVel;
}

void StateEstimateBase::updateJointStates() {
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
        rbdState_(6 + i) = hybridJointHandles_[i].getPosition();
        rbdState_(generalizedCoordinatesNum_ + 6 + i) = hybridJointHandles_[i].getVelocity();
    }
}

void StateEstimateBase::updateArmEE() {
    armEeKinematics_->setPinocchioInterface(pinocchioInterface_);

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    size_t actuatedDofNum = info_.actuatedDofNum;

    vector_t qPino(generalizedCoordinatesNum_);

    qPino.head<3>() = rbdState_.segment<3>(3);
    qPino.segment<3>(3) = rbdState_.head<3>();
    qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

    pinocchio::forwardKinematics(model, data, qPino);
    pinocchio::updateFramePlacements(model, data);

    const auto armEePos = armEeKinematics_->getPosition(vector_t());

    const size_t frame_idx = model.getBodyId(armEeKinematics_->getIds()[0]);
    const auto armEeOriQuat = matrixToQuaternion(data.oMf[frame_idx].rotation());

    rbdState_.segment<4>(2 * generalizedCoordinatesNum_ + 3) = vector_t(armEeOriQuat.coeffs());
    rbdState_.segment<3>(2 * generalizedCoordinatesNum_) = armEePos[0];
}

void StateEstimateBase::updateJointEffort() {
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
        rbdState_(2 * generalizedCoordinatesNum_ + 7 + 3 + i) = hybridJointHandles_[i].getEffort();
    }
}

void StateEstimateBase::updateEEForce(const Eigen::Vector3d &force) {
    // joint 6
//    const auto& model = pinocchioInterface_.getModel();
//    auto& data = pinocchioInterface_.getData();
//    const size_t frame_idx = model.getBodyId(armEeKinematics_->getIds()[0]);
//    const auto jointIndex = model.frames[frame_idx].parent;
//    const Eigen::Matrix3d rotationJointFrameToWorldFrame = data.oMi[jointIndex].rotation();
//    const scalar_t mass = 0.01;
//    vector3_t acc = pinocchio::getAcceleration(model, data, frame_idx, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    Eigen::Matrix3d rotationJointFrameToWorldFrame;
    rotationJointFrameToWorldFrame << 0, 0, -1, 0, -1, 0, -1, 0, 0;

    Eigen::Vector3d eeforce= rotationJointFrameToWorldFrame * force;
//    eeforce = eeforce + mass * acc;
//    eeforce(2) = eeforce(2) + mass * 9.81;
    for (size_t i = 0; i < 3; ++i) {
        rbdState_(2 * generalizedCoordinatesNum_ + 7 + i) = eeforce(i); // in frame ee
    }
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom, const ros::Time& time) {
    scalar_t publishRate = 100;
    if (lastPub_ + ros::Duration(1. / publishRate) < time) {
        if (odomPub_->trylock()) {
            odomPub_->msg_.pose = odom.pose;
            odomPub_->msg_.twist = odom.twist;
            odomPub_->unlockAndPublish();
        }
        if (posePub_->trylock()) {
            posePub_->msg_.pose = odom.pose;
            posePub_->unlockAndPublish();
        }
        lastPub_ = time;
    }
}

}