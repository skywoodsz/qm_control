#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/CommandLine.h>

#include <qm_controllers/ActionSet.h>
#include <qm_controllers/MarkerPosePublisher.h>

using namespace ocs2;
using namespace qm;
using namespace legged_robot;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "doorOpeningPlanner");
    if (argc < 2)
    {
        std::cout << "Usage:" << std::endl
                  << "1:grasp;  0:ungrasp;  2:standing_trot;  3:move to door;  4:stance; "
                  << "5:rotate handle; 6:rotate back; 7:push position; 8: final target";
        return 1;
    }
    int stage = std::stoi(argv[1]);

    ros::NodeHandle nh;
    marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    gait_pub = nh.advertise<std_msgs::String>("/gait_mode_cmd", 10);
    gait_mode_sub = nh.subscribe<std_msgs::String>("/gait_mode_cmd", 1, gaitCmdCallback);
    marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);

    ros::Rate loop_rate(freq);

    initial_pose = setInitPose(); // only initailized, not real initail pose
    // waiting for initial ee pose
    while (!received_curr_marker && ros::ok() && ros::master::check())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    marker_pose = initial_pose;

    // Get gait command file
    nh.getParam("/gaitCommandFile", gaitCommandFile);
    myGaitKeyboardPublisher gaitCommand(nh, gaitCommandFile, "legged_robot", true);

    // Enable finger position publishers
    for (int i = 0; i < 3; i++)
    {
        fingers[i] = nh.advertise<std_msgs::Float64>("/finger_" + std::to_string(i + 1) + "_position_controller/command", 50);
        // fingers[i + 3] = nh.advertise<std_msgs::Float64>("/finger_tip_" + std::to_string(i + 1) + "_position_controller/command", 50);
    }

    while (ros::ok())
    {
        if (stage == 1 || stage == 0)
        {
            grasp_door_handle(1.2, 1.22, !stage);
        }
        if (stage == 2)
        {
            switch_gait_pub(gaitCommand, "standing_trot");
        }
        if (stage == 3)
        {
            move_to(set_grasping_pose());
        }
        if (stage == 4)
        {
            switch_gait_pub(gaitCommand, "stance");
        }
        if (stage == 5)
        {
            double rotate_angle = 45 * M_PI / 180.0;
            bool finish = rotate_handle(rotate_angle);
            if (finish)
                break;
        }
        if (stage == 6)
        {
            double rotate_angle = -45 * M_PI / 180.0;
            bool finish = rotate_handle(rotate_angle, -0.2);
            if (finish)
                break;
        }
        if (stage == 7)
        {
            // geometry_msgs::Transform back_target = set_grasping_pose();
            // back_target.translation.x -= 0.3;
            // back_target.translation.z = 0.707;
            // move_to(back_target, 0.05);
            geometry_msgs::Transform back_target = set_grasping_pose();
            back_target.translation.x += 0.005;
            back_target.translation.z = 0.9;
            move_to(back_target, 0.15);
        }
        if (stage == 8)
        {
            // geometry_msgs::Transform back_target = set_grasping_pose();
            // back_target.translation.x -= 0.3;
            // back_target.translation.z = 0.707;
            // move_to(back_target, 0.05);
            geometry_msgs::Transform back_target = set_grasping_pose();
            back_target.translation.x += 0.005;
            back_target.translation.z = 1.06;
            move_to(back_target, 0.05);
        }
        if (stage == 9)
        {
            // geometry_msgs::Transform final_target = set_grasping_pose();
            // final_target.translation.x += 3;
            // final_target.translation.z = 0.707;
            // move_to(final_target, 0.25);
            double rotate_angle = 45 * M_PI / 180.0;
            bool finish = rotate_hinge(rotate_angle, 0.07, 0);
            if (finish)
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
