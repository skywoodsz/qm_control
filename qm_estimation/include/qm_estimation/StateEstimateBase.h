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
#include <tf2_ros/transform_broadcaster.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <qm_common/hardware_interface/ContactSensorInterface.h>
#include <qm_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>


namespace qm{
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase {
public:
    StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                      const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics,
                      std::vector<HybridJointHandle> hybridJointHandles, std::vector<ContactSensorHandle> contactSensorHandles,
                      hardware_interface::ImuSensorHandle imuSensorHandle);
    virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;
    size_t getMode();

protected:
    void updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angularVel);
    void updateLinear(const vector_t& pos, const vector_t& linearVel);
    void updateJointStates();
    void publishMsgs(const nav_msgs::Odometry& odom, const ros::Time& time);
    void updateArmEE();

    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    std::unique_ptr<PinocchioEndEffectorKinematics> armEeKinematics_;

    size_t generalizedCoordinatesNum_;
    vector_t rbdState_;

    const std::vector<HybridJointHandle> hybridJointHandles_;
    const std::vector<ContactSensorHandle> contactSensorHandles_;
    const hardware_interface::ImuSensorHandle imuSensorHandle_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
    ros::Time lastPub_;
    tf2_ros::TransformBroadcaster tf_br_;
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
