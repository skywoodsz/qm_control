//
// Created by skywoodsz on 2023/2/28.
//
// copy from: https://github.com/qiayuanliao/legged_control

#ifndef SRC_FROMTOPICEESTIMATE_H
#define SRC_FROMTOPICEESTIMATE_H

#include "qm_estimation/StateEstimateBase.h"

#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_buffer.h>

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
    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    ros::Subscriber sub_, force_sub_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
    realtime_tools::RealtimeBuffer<geometry_msgs::WrenchStamped> wrench_buffer_;
};

}


#endif //SRC_FROMTOPICEESTIMATE_H
