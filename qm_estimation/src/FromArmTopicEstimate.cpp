//
// Created by skywoodsz on 2023/3/10.
//

#include "qm_estimation/FromArmTopicEstimate.h"


namespace qm {
using namespace ocs2;
using namespace legged_robot;

FromArmTopicEstimate::FromArmTopicEstimate(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                           const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                           const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                                           const std::vector<HybridJointHandle> &hybridJointHandles,
                                           const std::vector<ContactSensorHandle> &contactSensorHandles,
                                           const hardware_interface::ImuSensorHandle &imuSensorHandle)
       : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics,
                           armEeKinematics, hybridJointHandles, contactSensorHandles, imuSensorHandle){

    auto jointStateCB = [&](const sensor_msgs::JointState::ConstPtr &msg)
    {
        joint_state_buffer_.writeFromNonRT(*msg);
        get_arm_state_flag_ = true;
    };

    auto odomCB = [&](const nav_msgs::Odometry::ConstPtr& msg)
    {
        buffer_.writeFromNonRT(*msg);
    };

    ros::NodeHandle nh;
    sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, odomCB);
    arm_joint_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateCB);
}

void FromArmTopicEstimate::updateArmJointStates(sensor_msgs::JointState jointState) {

    for (size_t i = hybridJointHandles_.size(); i < jointState.position.size(); ++i) {
        rbdState_(6 + i) = jointState.position[i];
        rbdState_(generalizedCoordinatesNum_ + 6 + i) = jointState.velocity[i];
    }

}

vector_t FromArmTopicEstimate::update(const ros::Time &time, const ros::Duration &period) {
    nav_msgs::Odometry odom = *buffer_.readFromRT();
    sensor_msgs::JointState jointState = *joint_state_buffer_.readFromRT();

    updateAngular(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                              odom.pose.pose.orientation.z),
                  Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
    updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                 Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

    updateJointStates();

    updateArmJointStates(jointState);

    updateArmEE();

    publishMsgs(odom, time);

    return rbdState_;
}


}