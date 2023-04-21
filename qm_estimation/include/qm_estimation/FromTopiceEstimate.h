//
// Created by skywoodsz on 2023/2/28.
//
// copy from: https://github.com/qiayuanliao/legged_control

#ifndef SRC_FROMTOPICEESTIMATE_H
#define SRC_FROMTOPICEESTIMATE_H

#include "qm_estimation/StateEstimateBase.h"

#include <sensor_msgs/JointState.h>
#include <qm_msgs/LegsState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class FromTopicStateEstimate : public StateEstimateBase {
public:
    FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                           const PinocchioEndEffectorKinematics& eeKinematics,
                           const PinocchioEndEffectorKinematics& armEeKinematics,
                           const std::vector<HybridJointHandle>& hybridJointHandles,
                           const std::vector<ContactSensorHandle>& contactSensorHandles,
                           const hardware_interface::ImuSensorHandle& imuSensorHandle);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

private:
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
    void publishLegState(const ros::Time& time, const ros::Duration& period);

    std::shared_ptr<realtime_tools::RealtimePublisher<qm_msgs::LegsState>> state_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> joint_pub_;
    ros::Time last_publish_;

    ros::Subscriber sub_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}


#endif //SRC_FROMTOPICEESTIMATE_H
