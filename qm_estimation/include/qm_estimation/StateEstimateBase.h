//
// Created by skywoodsz on 2023/2/28.
//
// copy from: https://github.com/qiayuanliao/legged_control

#ifndef SRC_STATEESTIMATEBASE_H
#define SRC_STATEESTIMATEBASE_H

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <qm_common/hardware_interface/ContactSensorInterface.h>
#include <qm_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase {
public:
    StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                      const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics);

    virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);
    virtual void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
    virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                           const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                           const matrix3_t& linearAccelCovariance);

    virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;

    size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

protected:
    void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
    void updateLinear(const vector_t& pos, const vector_t& linearVel);
    void updateArmEE(); // TODO: new
    void publishMsgs(const nav_msgs::Odometry& odom);

    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    std::unique_ptr<PinocchioEndEffectorKinematics> armEeKinematics_;

    bool offsetFlag_;
    vector3_t zyxOffset_ = vector3_t::Zero();
    vector3_t linearAccOffset_ = vector3_t::Zero();
    contact_flag_t contactFlag_{};
    Eigen::Quaternion<scalar_t> quat_;
    vector3_t angularVelLocal_, linearAccelLocal_;
    matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

    size_t generalizedCoordinatesNum_;
    vector_t rbdState_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
    ros::Time lastPub_;
};

template <typename T>
T square(T a) {
    return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

}


#endif //SRC_STATEESTIMATEBASE_H
