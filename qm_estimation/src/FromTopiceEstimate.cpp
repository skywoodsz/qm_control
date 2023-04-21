//
// Created by skywoodsz on 2023/2/28.
//
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

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
    state_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<qm_msgs::LegsState>>(nh, "/leg_states", 100);
    joint_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(nh, "/joint_states", 100);

    sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
    buffer_.writeFromNonRT(*msg);
}

void FromTopicStateEstimate::publishLegState(const ros::Time& time, const ros::Duration& period) {
    if (time < ros::Time(0.01))  // When simulate time reset
        last_publish_ = time;
    if (time - last_publish_ > ros::Duration(0.01))  // 100Hz
    {
        last_publish_ = time;

        // get contact flag
        contact_flag_t contactFlag;
        for (size_t i = 0; i < contactSensorHandles_.size(); ++i) {
            contactFlag[i] = contactSensorHandles_[i].isContact();
        }

        // get leg state
        eeKinematics_->setPinocchioInterface(pinocchioInterface_);
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();

        size_t actuatedDofNum = info_.actuatedDofNum;
        vector_t qPino(generalizedCoordinatesNum_);
        vector_t vPino(generalizedCoordinatesNum_);

        qPino.head<3>() = rbdState_.segment<3>(3);
        qPino.segment<3>(3) = rbdState_.head<3>();
        qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

        vPino.head<3>() = rbdState_.segment<3>(generalizedCoordinatesNum_ + 3);
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
                qPino.segment<3>(3), rbdState_.segment<3>(generalizedCoordinatesNum_));
        vPino.tail(info_.actuatedDofNum) = rbdState_.segment(generalizedCoordinatesNum_ + 6, actuatedDofNum);

        pinocchio::forwardKinematics(model, data, qPino, vPino);
        pinocchio::computeJointJacobians(model, data);
        pinocchio::updateFramePlacements(model, data);

        std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
        std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

        if (state_pub_->trylock())
        {
            for (int leg = 0; leg < 4; ++leg) {
                state_pub_->msg_.contact_state[leg] = contactFlag[leg];

                state_pub_->msg_.foot_pos[leg].x = posMeasured[leg].x();
                state_pub_->msg_.foot_pos[leg].y = posMeasured[leg].y();
                state_pub_->msg_.foot_pos[leg].z = posMeasured[leg].z();

                state_pub_->msg_.foot_vel[leg].x = velMeasured[leg].x();
                state_pub_->msg_.foot_vel[leg].y = velMeasured[leg].y();
                state_pub_->msg_.foot_vel[leg].z = velMeasured[leg].z();
            }
            state_pub_->unlockAndPublish();
        }
        if(joint_pub_->trylock())
        {
            joint_pub_->msg_.header.stamp = time;
            joint_pub_->msg_.position.resize(6);
            joint_pub_->msg_.velocity.resize(6);
            joint_pub_->msg_.effort.resize(6);
            joint_pub_->msg_.name.resize(6);
            vector_t arm_joint_pos = qPino.tail(6);
            vector_t arm_joint_vel = vPino.tail(6);
            for (int joint = 0; joint < 6; ++joint) {
                joint_pub_->msg_.position[joint] = arm_joint_pos(joint);
                joint_pub_->msg_.velocity[joint] = arm_joint_vel(joint);
                joint_pub_->msg_.effort[joint] = 0.;
            }
        }
        joint_pub_->unlockAndPublish();
    }
}

vector_t FromTopicStateEstimate::update(const ros::Time& time, const ros::Duration& period) {
    nav_msgs::Odometry odom = *buffer_.readFromRT();

    updateAngular(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                              odom.pose.pose.orientation.z),
                  Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));

    updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                 Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));
    
    updateJointStates();

    updateArmEE();

    publishMsgs(odom, time);
    publishLegState(time, period);

    return rbdState_;
}

}