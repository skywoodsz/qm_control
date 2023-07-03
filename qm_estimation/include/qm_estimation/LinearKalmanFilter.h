//
// Created by skywoodsz on 2023/6/15.
//

#ifndef SRC_LINEARKALMANFILTER_H
#define SRC_LINEARKALMANFILTER_H

#include "qm_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>

namespace qm{
using namespace ocs2;

class KalmanFilterEstimate : public StateEstimateBase{
public:
    KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

protected:
    void updateFromTopic();

    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    nav_msgs::Odometry getOdomMsg();

    vector_t feetHeights_;

    // Config
    scalar_t footRadius_ = 0.02;
    scalar_t imuProcessNoisePosition_ = 0.02;
    scalar_t imuProcessNoiseVelocity_ = 0.02;
    scalar_t footProcessNoisePosition_ = 0.002;
    scalar_t footSensorNoisePosition_ = 0.005;
    scalar_t footSensorNoiseVelocity_ = 0.05;
    scalar_t footHeightSensorNoise_ = 0.001;

private:
    Eigen::Matrix<scalar_t, 18, 1> xHat_;
    Eigen::Matrix<scalar_t, 12, 1> ps_;
    Eigen::Matrix<scalar_t, 12, 1> vs_;
    Eigen::Matrix<scalar_t, 18, 18> a_;
    Eigen::Matrix<scalar_t, 18, 18> q_;
    Eigen::Matrix<scalar_t, 18, 18> p_;
    Eigen::Matrix<scalar_t, 28, 28> r_;
    Eigen::Matrix<scalar_t, 18, 3> b_;
    Eigen::Matrix<scalar_t, 28, 18> c_;

    // Topic
    ros::Subscriber sub_;
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2::Transform world2odom_;
    std::string frameOdom_, frameGuess_;
    bool topicUpdated_;
};

}


#endif //SRC_LINEARKALMANFILTER_H
