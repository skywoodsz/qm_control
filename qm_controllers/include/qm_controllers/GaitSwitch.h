#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"
#include "qm_controllers/MarkerPosePublisher.h"

namespace ocs2
{
    namespace legged_robot
    {

        /** This class implements ModeSequence communication using ROS. */
        class myGaitKeyboardPublisher
        {
        public:
            myGaitKeyboardPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile, const std::string &robotName, bool verbose = false)
            {
                ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
                loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);
                modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

                gaitMap_.clear();
                for (const auto &gaitName : gaitList_)
                {
                    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
                }
                ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
            }

            /** Prints the command line interface and responds to user input. Function returns after one user input. */
            void getKeyboardCommand()
            {
                const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
                std::cout << commadMsg << ": ";

                auto shouldTerminate = []()
                { return !ros::ok() || !ros::master::check(); };
                const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

                if (commandLine.empty())
                {
                    return;
                }

                if (commandLine.size() > 1)
                {
                    std::cout << "WARNING: The command should be a single word." << std::endl;
                    return;
                }

                // lower case transform
                auto gaitCommand = commandLine.front();
                std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

                if (gaitCommand == "list")
                {
                    printGaitList(gaitList_);
                    return;
                }

                try
                {
                    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
                    modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
                    printGaitList(gaitList_);
                }
            }

            void switchGaitMode(std::string gait_mode)
            {
                // imitate ocs2_legged_robot_ros/src/gait/GaitKeyboardPublisher.cpp line 63
                auto shouldTerminate = []()
                { return !ros::ok() || !ros::master::check(); };
                const auto modeCommand = stringToWords(gait_mode);
                if (modeCommand.empty())
                {
                    return;
                }
                // lower case transform
                std::string gaitMode = modeCommand.front();
                std::transform(gaitMode.begin(), gaitMode.end(), gaitMode.begin(), ::tolower);

                try
                {
                    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitMode);
                    modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "Gait \"" << gaitMode << "\" not found.\n";
                }
            }

        private:
            /** Prints the list of available gaits. */
            void printGaitList(const std::vector<std::string> &gaitList) const
            {
                std::cout << "List of available gaits:\n";
                size_t itr = 0;
                for (const auto &s : gaitList)
                {
                    std::cout << "[" << itr++ << "]: " << s << "\n";
                }
                std::cout << std::endl;
            }

            std::vector<std::string> gaitList_;
            std::map<std::string, ModeSequenceTemplate> gaitMap_;

            ros::Publisher modeSequenceTemplatePublisher_;
        };

    } // namespace legged_robot
} // end of namespace ocs2

ros::Subscriber gait_mode_sub;

std::string sub_gait_mode;
std::string curr_gait;
bool is_converted = 0;
std::string gaitCommandFile;

void gaitCmdCallback(const std_msgs::String::ConstPtr &msg)
{
    sub_gait_mode = msg->data;
    if (sub_gait_mode != curr_gait)
    {
        std::cout<<"Command: Switch gait mode to: "<<sub_gait_mode.c_str()<<std::endl;
        is_converted = 1;
        curr_gait = sub_gait_mode;
        return;
    }
    is_converted = 0;
}
