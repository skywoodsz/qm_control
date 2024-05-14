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
                  << "1:grasp;  0:ungrasp;  2:rotate base;  3:move to door;  4:stance; "
                  << "5:rotate handle; 6:rotate back; 7: pull door; 8: move base";
        return 1;
    }
    int stage = std::stoi(argv[1]);

    ros::NodeHandle nh;
    marker_pose_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/marker_pose", 10);
    gait_pub = nh.advertise<std_msgs::String>("/gait_mode_cmd", 10);
    gait_mode_sub = nh.subscribe<std_msgs::String>("/gait_mode_cmd", 1, gaitCmdCallback);
    marker_feedback_sub = nh.subscribe<qm_msgs::ee_state>("/qm_mpc_observation_ee_state", 1, eeStateCallback);
    base_state_sub=nh.subscribe<ocs2_msgs::mpc_observation>("/qm_mpc_observation", 1, baseStateCallback);
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
            grasp_door_handle(1.25, 1.21, !stage);
        }
        if (stage == 2)
        {
            // switch_gait_pub(gaitCommand, "standing_trot");
            // break;
            double rotate_angle = -180 * M_PI / 180.0;
            bool finish = rotate_base(rotate_angle, -0.2);
            if (finish)
                break;
        }
        if (stage == 3)
        {
            move_to(set_grasping_pose(0), 0.25);
        }
        if (stage == 4)
        {
            switch_gait_pub(gaitCommand, "stance");
            break;
        }
        if (stage == 5)
        {
            double rotate_angle = 45 * M_PI / 180.0;
            bool finish = rotate_handle(rotate_angle, 0.2, 1);
            if (finish)
                break;
        }
        if (stage == 6)
        {
            double rotate_angle = -45 * M_PI / 180.0;
            bool finish = rotate_handle(rotate_angle, -0.2, 1);
            if (finish)
                break;
        }
        if (stage == 7)
        {
            double rotate_angle = 60 * M_PI / 180.0;
            bool finish = rotate_hinge(rotate_angle, 0.1, 0);
            if (finish)
                break;
        }
        if (stage == 8)
        {
            double rotate_angle = 0.1 * M_PI / 180.0;
            bool finish = rotate_hinge(rotate_angle, 0.1, 0);
            if (finish)
                break;
        }

        if (stage == 9)
        {
            geometry_msgs::Transform point1;
            double yaw=-base_state.angular.z;
            double dx=0, dy=0.1;
            
            point1.translation.x=base_state.linear.x+dx*cos(yaw)-dy*sin(yaw);
            point1.translation.y=base_state.linear.y-dy*cos(yaw)-dx*sin(yaw);
            point1.translation.z=1;
            point1.rotation.x=-0.649;
            point1.rotation.y=0.279;
            point1.rotation.z=0.649;
            point1.rotation.w=0.279;
            move_to(point1);
        }

        if (stage == 10)
        {
            geometry_msgs::Transform point1;
            double yaw=-base_state.angular.z;
            double dx=5, dy=-0.1;
            
            point1.translation.x=base_state.linear.x+dx*cos(yaw)-dy*sin(yaw);
            point1.translation.y=base_state.linear.y-dy*cos(yaw)-dx*sin(yaw);
            point1.translation.z=1;
            point1.rotation.x=-0.649;
            point1.rotation.y=0.279;
            point1.rotation.z=0.649;
            point1.rotation.w=0.279;
            move_to(point1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
