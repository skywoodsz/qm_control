//
// Created by skywoodsz on 2023/2/28.
//
// copy from: https://github.com/qiayuanliao/legged_control

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "qm_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>


namespace qm{
using namespace ocs2;
using namespace legged_robot;

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()),
          armEeKinematics_(armEeKinematics.clone()),
          rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum + 7)) // + ee pose
{
    offsetFlag_ = false;
    generalizedCoordinatesNum_ = info_.generalizedCoordinatesNum;

    ros::NodeHandle nh;
    odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));
    posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));

    armEeKinematics_->setPinocchioInterface(pinocchioInterface_);
}

void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
    rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
    rbdState_.segment(6 + generalizedCoordinatesNum_, info_.actuatedDofNum) = jointVel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
    // Offset
    if(!offsetFlag_)
    {
        offsetFlag_ = true;
        zyxOffset_ = quatToZyx(quat);
        linearAccOffset_ = linearAccelLocal;
    }

    vector3_t zyx = quatToZyx(quat) - zyxOffset_;
    vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
    updateAngular(zyx, angularVelGlobal);

    quat_ = quat * getQuaternionFromEulerAnglesZyx<scalar_t>(-zyxOffset_);
    angularVelLocal_ = angularVelLocal;
    linearAccelLocal_ = linearAccelLocal;
    orientationCovariance_ = orientationCovariance;
    angularVelCovariance_ = angularVelCovariance;
    linearAccelCovariance_ = linearAccelCovariance;
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
    rbdState_.segment<3>(0) = zyx;
    rbdState_.segment<3>(generalizedCoordinatesNum_) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
    rbdState_.segment<3>(3) = pos;
    rbdState_.segment<3>(generalizedCoordinatesNum_ + 3) = linearVel;
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

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
    ros::Time time = odom.header.stamp;
    scalar_t publishRate = 100;
    if (lastPub_ + ros::Duration(1. / publishRate) < time) {
        lastPub_ = time;
        if (odomPub_->trylock()) {
            odomPub_->msg_ = odom;
            odomPub_->unlockAndPublish();
        }
        if (posePub_->trylock()) {
            posePub_->msg_.header = odom.header;
            posePub_->msg_.pose = odom.pose;
            posePub_->unlockAndPublish();
        }
    }
}

}