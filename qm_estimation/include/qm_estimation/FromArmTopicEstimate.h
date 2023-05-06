//
// Created by skywoodsz on 2023/3/10.
//

#ifndef SRC_FROMARMTOPICESTIMATE_H
#define SRC_FROMARMTOPICESTIMATE_H

#include "qm_estimation/StateEstimateBase.h"

#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>

namespace qm {
using namespace ocs2;
using namespace legged_robot;

class FromArmTopicEstimate : public StateEstimateBase {
public:
    FromArmTopicEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                           const PinocchioEndEffectorKinematics& eeKinematics,
                           const PinocchioEndEffectorKinematics& armEeKinematics,
                           const std::vector<HybridJointHandle>& hybridJointHandles,
                           const std::vector<ContactSensorHandle>& contactSensorHandles,
                           const hardware_interface::ImuSensorHandle& imuSensorHandle);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

    bool get_arm_state_flag_;

private:
    void updateArmJointStates(sensor_msgs::JointState jointState);

    ros::Subscriber sub_, arm_joint_sub_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
    realtime_tools::RealtimeBuffer<sensor_msgs::JointState> joint_state_buffer_;

};



}

#endif //SRC_FROMARMTOPICESTIMATE_H
