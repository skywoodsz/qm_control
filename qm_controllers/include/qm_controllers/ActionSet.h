#pragma once

#include "qm_controllers/MarkerPosePublisher.h"
#include "qm_controllers/GaitSwitch.h"

#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/CommandLine.h>

using namespace ocs2;
using namespace qm;
using namespace legged_robot;

// ros param
double freq = 10;
double step_time = 1.0 / freq;
// fingers[0]~[5]: finger1, finger2, finger3, finger_tip1, finger_tip2, finger_tip3;
ros::Publisher fingers[3];
ros::Publisher gait_pub;
// grasp action
std_msgs::Float64 fingers_val[3];

/**@brief grasp door handle
 * @param finger1 finger_1 joint position
 * @param finger2 finger_2 joint position
 * @param ungrasp 0: grasp 1: release*/
void grasp_door_handle(double finger1, double finger2, bool ungrasp = 0)
{
    // ungrasp
    if (ungrasp)
    {
        for (int i = 0; i < 3; i++)
        {
            fingers_val[i].data = 0;
            fingers[i].publish(fingers_val[i]);
        }
        return;
    }

    // grasp
    fingers_val[0].data += 0.02;
    if (fingers_val[0].data >= finger1)
        fingers_val[0].data = finger1;
    fingers_val[1].data += 0.02;
    if (fingers_val[1].data >= finger2)
        fingers_val[1].data = finger2;
    fingers_val[2].data = 0;
    for (int i = 0; i < 3; i++)
    {
        fingers[i].publish(fingers_val[i]);
    }
}

// switch gait mode
void switch_gait_pub(myGaitKeyboardPublisher myGaitCommand, std::string gaitName)
{
    std_msgs::String init_gait;
    init_gait.data = gaitName;
    gait_pub.publish(init_gait);
    if (is_converted)
    {
        myGaitCommand.switchGaitMode(sub_gait_mode);
    }
}

// subcribe grasping point
geometry_msgs::Transform set_grasping_pose(int mode = 0)
{
    geometry_msgs::Vector3 grasp_point;
    geometry_msgs::Quaternion grasp_quat;
    if (mode == 0) // push
    {
        // tf2::Vector3 grasp_point_(2.99, 0.175, 1.068);
        tf2::Vector3 grasp_point_(3, 0.188, 1.068);
        // tf2::Vector3 grasp_point_(2.98, 0.28, 1.07);
        grasp_point = tf2::toMsg(grasp_point_);
        // tf2::Quaternion grasp_quat_(0, 1 / sqrt(2), 0, 1 / sqrt(2)); // rotate 90 degree around world_Y
        tf2::Quaternion grasp_quat_(0.5,-0.5, 0.5, -0.5);
        grasp_quat_ = grasp_quat_.normalize();
        grasp_quat = tf2::toMsg(grasp_quat_);
    }
    else // pull
    {
        tf2::Vector3 grasp_point_(-2.9, 0.29, 1.1);
        grasp_point = tf2::toMsg(grasp_point_);
        tf2::Quaternion grasp_quat_(-1 / sqrt(2), 0, 1 / sqrt(2), 0); // rotate -90 degree around world_Y
        grasp_quat_ = grasp_quat_.normalize();
        grasp_quat = tf2::toMsg(grasp_quat_);
    }

    geometry_msgs::Transform grasping_pose;
    grasping_pose.translation = grasp_point;
    grasping_pose.rotation = grasp_quat;
    return grasping_pose;
}

// move to specified point
void move_to(geometry_msgs::Transform target_pose, double v_max = 0.15)
{
    geometry_msgs::Vector3 target_position;
    target_position.x = target_pose.translation.x;
    target_position.y = target_pose.translation.y;
    target_position.z = target_pose.translation.z;

    marker_pose.pose.orientation = target_pose.rotation;
    markerPosePosControl(marker_pose, target_position, step_time, v_max, 1);
}

// subscribe handle frame for rotation
geometry_msgs::Transform set_handle_frame(int mode = 0)
{
    geometry_msgs::Vector3 handle_point;
    if (mode == 0) // push
    {
        tf2::Vector3 handle_point_(3.0465, 0.2922, 1.068);
        handle_point = tf2::toMsg(handle_point_);
    }
    else // pull
    {
        tf2::Vector3 handle_point_(-3.06, 0.35, 1.068);
        handle_point = tf2::toMsg(handle_point_);
    }

    tf2::Quaternion handle_quat_(0, 1 / sqrt(2), 0, 1 / sqrt(2)); // rotate 90 degree around world_Y
    handle_quat_ = handle_quat_.normalize();
    geometry_msgs::Quaternion handle_quat = tf2::toMsg(handle_quat_);

    geometry_msgs::Transform handle_frame;
    handle_frame.translation = handle_point;
    handle_frame.rotation = handle_quat;
    return handle_frame;
}

// rotate handle, angle:radian, omega:rad/s
bool rotate_handle(double angle, double omega = 0.2, int mode = 0)
{
    geometry_msgs::Transform handle_frame = set_handle_frame(mode);
    bool finished = markerPoseAngularPosControl(
        marker_pose, handle_frame.translation, handle_frame.rotation,
        step_time, omega, "z", angle);
    return finished;
}

geometry_msgs::Transform set_hinge_frame(int mode = 0)
{
    geometry_msgs::Vector3 hinge_point;
    if (mode == 0) // push
    {
        tf2::Vector3 hinge_point_(3.06, -0.5, 1);
        hinge_point = tf2::toMsg(hinge_point_);
    }
    else // pull
    {
        tf2::Vector3 hinge_point_(-3.06, -0.45, 1);
        hinge_point = tf2::toMsg(hinge_point_);
    }

    tf2::Quaternion hinge_quat_(0, 0, 0, 1);
    hinge_quat_ = hinge_quat_.normalize();
    geometry_msgs::Quaternion hinge_quat = tf2::toMsg(hinge_quat_);

    geometry_msgs::Transform hinge_frame;
    hinge_frame.translation = hinge_point;
    hinge_frame.rotation = hinge_quat;
    return hinge_frame;
}

// rotate hinge, angle:radian, omega:rad/s
bool rotate_hinge(double angle, double omega = 0.1, int mode = 0)
{
    geometry_msgs::Transform hinge_frame = set_hinge_frame(mode);
    bool finished = markerPoseAngularPosControl(
        marker_pose, hinge_frame.translation, hinge_frame.rotation,
        step_time, omega, "z", angle);
    return finished;
}

bool rotate_base(double angle, double omega = 0.1)
{
    geometry_msgs::Transform base_frame;
    base_frame.translation.x=0;
    base_frame.translation.y=0.175;
    base_frame.translation.z=0;
    base_frame.rotation.x=0;
    base_frame.rotation.y=0;
    base_frame.rotation.z=0;
    base_frame.rotation.w=1;
    bool finished = markerPoseAngularPosControl(
        marker_pose, base_frame.translation, base_frame.rotation,
        step_time, omega, "z", angle);
    return finished;
}