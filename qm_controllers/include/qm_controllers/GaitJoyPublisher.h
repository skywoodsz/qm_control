//
// Created by skywoodsz on 2023/6/15.
//

#ifndef SRC_GAITJOYPUBLISHER_H
#define SRC_GAITJOYPUBLISHER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class GaitJoyPublisher{
public:
    GaitJoyPublisher(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName, bool verbose = false);
    void cmdVelCallback(const sensor_msgs::Joy::ConstPtr& msg);
private:
    ::ros::Subscriber cmdVelSub_;
    std::vector<std::string> gaitList_;
    std::map<std::string, ModeSequenceTemplate> gaitMap_;
    std::string lastGaitCommand_;

    ros::Publisher modeSequenceTemplatePublisher_;
};

}

#endif //SRC_GAITJOYPUBLISHER_H
