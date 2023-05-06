//
// Created by skywoodsz on 2023/2/28.
//

#include "qm_estimation/FromTopiceEstimate.h"



namespace qm{
using namespace ocs2;
using namespace legged_robot;

FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics,
                                               const PinocchioEndEffectorKinematics& armEeKinematics,
                                               const std::vector<HybridJointHandle>& hybridJointHandles,
                                               const std::vector<ContactSensorHandle>& contactSensorHandles,
                                               const hardware_interface::ImuSensorHandle& imuSensorHandle)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics, armEeKinematics, hybridJointHandles, contactSensorHandles,
                            imuSensorHandle) {
    ros::NodeHandle nh;
    sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
    force_sub_ = nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_topic", 10, &FromTopicStateEstimate::forceCallback, this);
}

void FromTopicStateEstimate::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
    wrench_buffer_.writeFromNonRT(*msg);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
    buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& time, const ros::Duration& /*period*/) {
    nav_msgs::Odometry odom = *buffer_.readFromRT();
    geometry_msgs::WrenchStamped wrench = *wrench_buffer_.readFromRT();

    updateAngular(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                              odom.pose.pose.orientation.z),
                  Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
    updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                 Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));
    
    updateJointStates();

    updateArmEE();

    updateEEForce(Eigen::Vector3d(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z));

    updateJointEffort();

    publishMsgs(odom, time);

    return rbdState_;
}

}