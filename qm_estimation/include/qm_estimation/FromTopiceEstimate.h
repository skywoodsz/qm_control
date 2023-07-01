//
// Created by skywoodsz on 2023/2/28.
//
// copy from: https://github.com/qiayuanliao/legged_control

#ifndef SRC_FROMTOPICEESTIMATE_H
#define SRC_FROMTOPICEESTIMATE_H

#include "qm_estimation/StateEstimateBase.h"

#include <realtime_tools/realtime_buffer.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class FromTopicStateEstimate : public StateEstimateBase {
public:
    FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                           const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

private:
    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber sub_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}


#endif //SRC_FROMTOPICEESTIMATE_H
