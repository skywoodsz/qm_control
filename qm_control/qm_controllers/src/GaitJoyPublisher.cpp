//
// Created by skywoodsz on 2023/6/15.
//

#include "qm_controllers/GaitJoyPublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

GaitJoyPublisher::GaitJoyPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile,
                                   const std::string &robotName, bool verbose) {
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
    loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

    modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

    gaitMap_.clear();
    for (const auto& gaitName : gaitList_) {
        gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
    }
    lastGaitCommand_ = "stance";
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");

    cmdVelSub_ = nodeHandle.subscribe<sensor_msgs::Joy>("joy", 1, &GaitJoyPublisher::cmdVelCallback, this);
}

void GaitJoyPublisher::cmdVelCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    sensor_msgs::Joy joy_msg = *msg;

    std::string gaitCommand;

    // trot
    if(joy_msg.buttons[4] + joy_msg.buttons[0] >= 2)
    {
        gaitCommand = "trot";
    }
    // stance
    if(joy_msg.buttons[4] + joy_msg.buttons[1] >= 2)
    {
        gaitCommand = "stance";
    }
    if(joy_msg.buttons[1] + joy_msg.buttons[0] >= 1 && lastGaitCommand_ != gaitCommand)
    {
        try {
            ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
            modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
            lastGaitCommand_ = gaitCommand;
        } catch (const std::out_of_range& e) {
            std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
        }
    }
}

}