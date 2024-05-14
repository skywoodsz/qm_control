#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"
#include "qm_controllers/MarkerPosePublisher.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

ros::Publisher marker_pose_pub;
ros::Subscriber marker_feedback_sub, marker_vel_sub,base_state_sub;

const int f = 10;
// position delta between obs & truth
const float delta_x = -0.003;
const float delta_y = -0.0015;
const float delta_z = 0.011;
// velocity control
geometry_msgs::Twist marker_vel;
ocs2_msgs::mpc_observation base_state_;
geometry_msgs::Twist base_state;
double pose_x_vel = 0, pose_y_vel = 0, pose_z_vel = 0;
visualization_msgs::InteractiveMarkerFeedback initial_pose;
visualization_msgs::InteractiveMarkerFeedback marker_pose;
bool received_curr_marker = 0;

void eeStateCallback(const qm_msgs::ee_state::ConstPtr &obs)
{
    initial_pose.pose.position.x = obs->state[0] + delta_x;
    initial_pose.pose.position.y = obs->state[1] + delta_y;
    initial_pose.pose.position.z = obs->state[2] + delta_z;

    initial_pose.pose.orientation.x = obs->state[3];
    initial_pose.pose.orientation.y = obs->state[4];
    initial_pose.pose.orientation.z = obs->state[5];
    initial_pose.pose.orientation.w = obs->state[6];

    received_curr_marker = 1;
}
void markerVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    marker_vel = *msg;
}
void baseStateCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
{
    base_state_ = *msg;
    base_state.linear.x=base_state_.state.value[6];
    base_state.linear.y=base_state_.state.value[7];
    base_state.linear.z=base_state_.state.value[8];
    base_state.angular.z=base_state_.state.value[9];
    base_state.angular.y=base_state_.state.value[10];
    base_state.angular.x=base_state_.state.value[11];
    
}

visualization_msgs::InteractiveMarkerFeedback setInitPose()
{
    visualization_msgs::InteractiveMarkerFeedback init_pose;
    init_pose.pose.position.x = 0.55;
    init_pose.pose.position.y = 0.175;
    init_pose.pose.position.z = 0.707;
    init_pose.pose.orientation.x = 0.0;
    init_pose.pose.orientation.y = sqrt(0.5);
    init_pose.pose.orientation.z = 0.0;
    init_pose.pose.orientation.w = sqrt(0.5);
    return init_pose;
}
// position vel control
void markerPoseVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    double step_time, geometry_msgs::Vector3 linear_vel)
{
    double v_x = linear_vel.x;
    double v_y = linear_vel.y;
    double v_z = linear_vel.z;
    curr.pose.position.x += v_x * step_time;
    curr.pose.position.y += v_y * step_time;
    curr.pose.position.z += v_z * step_time;
    marker_pose_pub.publish(curr);
}
// set linear_vel in linear_move, return 3 positive vel
geometry_msgs::Vector3 setLinearVel(double dx, double dy, double dz, double max_v)
{
    dx=fabs(dx);
    dy=fabs(dy);
    dz=fabs(dz);
    geometry_msgs::Vector3 linear_v;
    double d_max = std::max(dx, std::max(dy, dz));
    double k = max_v / d_max;
    linear_v.x = dx * k;
    linear_v.y = dy * k;
    linear_v.z = dz * k;
    // std::cout<<linear_v;
    return linear_v;
}
/**@brief position control
 * @param max_vel max(vx,vy,vz), would better not more than 0.2 (m/s)
 * @param order 0: move towards target in a straight path; 1:move along Z axis firstly*/
void markerPosePosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 target,
    double step_time, double max_vel, int order = 0)
{
    double x = curr.pose.position.x, y = curr.pose.position.y, z = curr.pose.position.z;
    double dx = target.x - x, dy = target.y - y, dz = target.z - z;
    if (fabs(dx) < 0.005 && fabs(dy) < 0.005 && fabs(dz) < 0.005)
    {
        ROS_INFO("Reached target");
        return;
    }

    geometry_msgs::Vector3 linear_vel;
    if (order == 0)
    {
        linear_vel = setLinearVel(dx, dy, dz, max_vel);
    }
    else if (order == 1)
    {
        if (fabs(dz) > 0.005)
        {
            linear_vel.x = 0;
            linear_vel.y = 0;
            linear_vel.z = max_vel;
        }
        else
        {
            linear_vel = setLinearVel(dx, dy, dz, max_vel);
        }
    }
    double step_x,step_y,step_z;
    if(dx>=0)
        step_x=std::min(linear_vel.x * step_time, dx);
    else
        step_x=std::max(-linear_vel.x * step_time, dx);
    if(dy>=0)
        step_y=std::min(linear_vel.y * step_time, dy);
    else
        step_y=std::max(-linear_vel.y * step_time, dy);
    if(dz>=0)
        step_z=std::min(linear_vel.z * step_time, dz);
    else
        step_z=std::max(-linear_vel.z * step_time, dz);
    curr.pose.position.x += step_x;
    curr.pose.position.y += step_y;
    curr.pose.position.z += step_z;
    marker_pose_pub.publish(curr);
}

// compute relative (x,y,z) of marker pose after rotation
tf2::Vector3 translationRotate(const visualization_msgs::InteractiveMarkerFeedback relative_position,
                               double step_angle, std::string rotation_axis)
{
    double x = relative_position.pose.position.x;
    double y = relative_position.pose.position.y;
    double z = relative_position.pose.position.z;

    if (rotation_axis == "x")
    {
        double new_y = y * cos(step_angle) - z * sin(step_angle);
        double new_z = y * sin(step_angle) + z * cos(step_angle);
        y = new_y;
        z = new_z;
    }
    else if (rotation_axis == "y")
    {
        double new_x = x * cos(step_angle) + z * sin(step_angle);
        double new_z = (-x) * sin(step_angle) + z * cos(step_angle);
        x = new_x;
        z = new_z;
    }
    else if (rotation_axis == "z")
    {
        double new_x = x * cos(step_angle) - y * sin(step_angle);
        double new_y = x * sin(step_angle) + y * cos(step_angle);
        x = new_x;
        y = new_y;
    }
    std::cout << "relative position: " << std::endl
              << relative_position.pose.position;

    tf2::Vector3 res_point(x, y, z);
    return res_point;
}

// compute (qx,qy,qz,qw) of marker pose after rotation, without center frame rotation
tf2::Quaternion quatRotate(const visualization_msgs::InteractiveMarkerFeedback curr,
                           const geometry_msgs::Vector3 center_point,
                           double step_angle, std::string rotation_axis)
{
    // calculate current marker's tf in world
    tf2::Vector3 translation_of_curr(curr.pose.position.x, curr.pose.position.y, curr.pose.position.z);
    tf2::Quaternion curr_quat_inW(curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w);
    tf2::Transform tf_of_curr_inW(curr_quat_inW, translation_of_curr);

    // center frame's origin in world
    tf2::Vector3 translation_of_C_O;
    tf2::fromMsg(center_point, translation_of_C_O);
    // center frame's quat in world, default is as same as world
    tf2::Quaternion quat_of_CinW(0, 0, 0, 1);
    // center frame's tf in world
    tf2::Transform tf_of_CinW(quat_of_CinW, translation_of_C_O);

    // calculate current marker's tf in center frame
    tf2::Transform tf_of_curr_inC = tf_of_CinW.inverse() * tf_of_curr_inW;

    // rotate marker according to RPY of curr in center frame
    tf2::Quaternion tmp = tf_of_curr_inC.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(tmp).getRPY(roll, pitch, yaw);
    if (rotation_axis == "x")
        roll += step_angle;
    else if (rotation_axis == "y")
        pitch += step_angle;
    else if (rotation_axis == "z")
        yaw += step_angle;
    else
        ROS_ERROR("NO AXIS SPECIFIED!");
    std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;

    // change tmp
    tmp.setRPY(roll, pitch, yaw);
    // new marker's tf in center frame
    tf2::Transform tf_of_new_inC;
    tf_of_new_inC.setRotation(tmp);
    // calculate new marker's tf in world
    tf2::Transform tf_of_new_inW = tf_of_CinW * tf_of_new_inC;
    return tf_of_new_inW.getRotation();
}

// compute (x,y,z,qx,qy,qz,qw) of marker pose after rotation on any axis
tf2::Transform poseRotate(const visualization_msgs::InteractiveMarkerFeedback curr,
                          const geometry_msgs::Vector3 center_point,
                          const geometry_msgs::Quaternion center_frame_quat,
                          double step_angle, std::string rotation_axis)
{
    // get current marker's tf in world
    tf2::Vector3 translation_of_curr(curr.pose.position.x, curr.pose.position.y, curr.pose.position.z);
    tf2::Quaternion curr_quat_inW(curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w);
    tf2::Transform tf_of_curr_inW(curr_quat_inW, translation_of_curr);

    // center frame's origin in world
    tf2::Vector3 translation_of_C_O;
    tf2::fromMsg(center_point, translation_of_C_O);
    // center frame's quat in world, default is as same as world
    tf2::Quaternion quat_of_CinW;
    tf2::fromMsg(center_frame_quat, quat_of_CinW);
    // center frame's tf in world
    tf2::Transform tf_of_CinW(quat_of_CinW, translation_of_C_O);

    // calculate current marker's tf in center frame
    tf2::Transform tf_of_curr_inC = tf_of_CinW.inverse() * tf_of_curr_inW;

    // translation
    visualization_msgs::InteractiveMarkerFeedback curr_inC;
    curr_inC.pose.position.x = tf_of_curr_inC.getOrigin().x();
    curr_inC.pose.position.y = tf_of_curr_inC.getOrigin().y();
    curr_inC.pose.position.z = tf_of_curr_inC.getOrigin().z();
    tf2::Vector3 new_trans_inC = translationRotate(curr_inC, step_angle, rotation_axis);

    tf2::Transform tf_of_new_inC, tf_of_new_inW;
    tf_of_new_inC.setOrigin(new_trans_inC);
    // calculate new translation after rotation
    tf_of_new_inW = tf_of_CinW * tf_of_new_inC;
    tf2::Vector3 trans_of_new_inW = tf_of_new_inW.getOrigin();

    // rotate marker according to RPY of curr in center frame
    tf2::Quaternion tmp = tf_of_curr_inC.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(tmp).getRPY(roll, pitch, yaw, 2);
    if (rotation_axis == "x")
        roll += step_angle;
    else if (rotation_axis == "y")
        pitch += step_angle;
    else if (rotation_axis == "z")
        yaw += step_angle;
    else
        ROS_ERROR("NO AXIS SPECIFIED!");
    std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;

    tmp.setRPY(roll, pitch, yaw);
    // new marker's tf in center frame
    tf_of_new_inC.setRotation(tmp);
    // calculate new marker's tf in world
    tf_of_new_inW = tf_of_CinW * tf_of_new_inC;

    // set new marker's translation
    tf_of_new_inW.setOrigin(trans_of_new_inW);
    return tf_of_new_inW;
}

/**@brief angular velocity control with center frame quaternion
 * @note won't stop; if occur gimbal problem, try to change Y-axis of center frame
 * @param center_point the origin of rotation center frame
 * @param center_frame_quat the quaternion of rotation center frame
 * @param rotation_axis axis of rotation center frame
 * @param angular_velocity right-hand rule (rad/s)*/
void markerPoseAngularVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const geometry_msgs::Quaternion center_frame_quat,
    double step_time, double angular_velocity, std::string rotation_axis,
    bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    tf2::Vector3 res_translation;
    tf2::Quaternion res_quat;
    tf2::Transform res_tf;
    // rotate (x,y,z) and (qx,qy,qz,qw) together
    res_tf = poseRotate(curr, center_point, center_frame_quat, step_angle, rotation_axis);
    res_translation = res_tf.getOrigin();

    // if you want to rotate marker orientation
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = res_tf.getRotation();
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}

/**@brief angular velocity control without center frame quaternion
 * @note won't stop; if occur gimbal problem, try to change Y-axis of center frame
 * @param center_point the origin of rotation center frame
 * @param rotation_axis axis of rotation center frame
 * @param angular_velocity right-hand rule (rad/s)*/
void markerPoseAngularVelControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    double step_time, double angular_velocity, std::string rotation_axis,
    bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    tf2::Vector3 relative_translation, res_translation;
    tf2::Quaternion res_quat;
    // translation of new marker in center_point_frame
    visualization_msgs::InteractiveMarkerFeedback relative_curr;
    relative_curr.pose.position.x = curr.pose.position.x - center_point.x;
    relative_curr.pose.position.y = curr.pose.position.y - center_point.y;
    relative_curr.pose.position.z = curr.pose.position.z - center_point.z;
    relative_translation = translationRotate(relative_curr, step_angle, rotation_axis);
    // transform to world frame
    res_translation.setX(relative_translation.x() + center_point.x);
    res_translation.setY(relative_translation.y() + center_point.y);
    res_translation.setZ(relative_translation.z() + center_point.z);

    // if you want to rotate marker orientation together
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = quatRotate(curr, center_point, step_angle, rotation_axis);
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}

/**@brief angular position control with center frame quaternion
 * @note will stop if reach target angle, then set is_rotated=0;
 * @note if occur gimbal problem, try to change Y-axis of center frame
 * @param center_point the origin of rotation center frame
 * @param rotation_axis axis of rotation center frame
 * @param delta_angle right-hand rule (radian)
 * @param angular_velocity right-hand rule (rad/s)
 * @return if finished, return 1*/
bool markerPoseAngularPosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const geometry_msgs::Quaternion center_frame_quat,
    const double step_time, const double angular_velocity, const std::string rotation_axis,
    const double delta_angle, bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;
    bool finished=0;

    // check if marker has reached target angle
    static double accumulated_angle = 0;
    accumulated_angle += step_angle;
    if (fabs(accumulated_angle) > fabs(delta_angle))
    {
        step_angle = delta_angle - (accumulated_angle - step_angle);
        finished=1;
    }
    std::cout << "acc_angle: " << step_angle << std::endl;

    tf2::Vector3 res_translation;
    tf2::Quaternion res_quat;
    tf2::Transform res_tf;
    // rotate (x,y,z) and (qx,qy,qz,qw) together
    res_tf = poseRotate(curr, center_point, center_frame_quat, step_angle, rotation_axis);
    res_translation = res_tf.getOrigin();

    // if you want to rotate marker orientation
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = res_tf.getRotation();
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
    if(finished)
    {
        accumulated_angle=0;
        return finished;
    }
    return 0;
}

/**@brief angular position control without center frame quaternion
 * @note will stop if reach target angle, then set is_rotated=0
 * @note if occur gimbal problem, try to change Y-axis of center frame
 * @param center_point the origin of rotation center frame
 * @param delta_angle right-hand rule (radian)
 * @param angular_velocity right-hand rule (rad/s)*/
void markerPoseAngularPosControl(
    visualization_msgs::InteractiveMarkerFeedback &curr,
    const geometry_msgs::Vector3 center_point,
    const double step_time, const double angular_velocity, const std::string rotation_axis,
    const double delta_angle, bool &is_rotated, bool rotate_orientation = 1)
{
    // rotation angle increment in every step
    double step_angle = angular_velocity * step_time;

    // check if marker has reached target angle
    static double accumulated_angle = 0;
    accumulated_angle += step_angle;
    if (accumulated_angle >= delta_angle)
    {
        step_angle = delta_angle - (accumulated_angle - step_angle);
        is_rotated = 0;
        accumulated_angle = 0;
    }
    std::cout << "acc_angle: " << accumulated_angle << std::endl;

    tf2::Vector3 relative_translation, res_translation;
    tf2::Quaternion res_quat;
    // translation of new marker in center_point_frame
    visualization_msgs::InteractiveMarkerFeedback relative_curr;
    relative_curr.pose.position.x = curr.pose.position.x - center_point.x;
    relative_curr.pose.position.y = curr.pose.position.y - center_point.y;
    relative_curr.pose.position.z = curr.pose.position.z - center_point.z;
    relative_translation = translationRotate(relative_curr, step_angle, rotation_axis);
    // transform to world frame
    res_translation.setX(relative_translation.x() + center_point.x);
    res_translation.setY(relative_translation.y() + center_point.y);
    res_translation.setZ(relative_translation.z() + center_point.z);

    // if you want to rotate marker orientation together
    if (rotate_orientation)
    {
        std::cout << "rotate orientation" << std::endl;
        res_quat = quatRotate(curr, center_point, step_angle, rotation_axis);
        curr.pose.orientation.x = res_quat.x();
        curr.pose.orientation.y = res_quat.y();
        curr.pose.orientation.z = res_quat.z();
        curr.pose.orientation.w = res_quat.w();
    }
    std::cout << std::endl;
    curr.pose.position.x = res_translation.x();
    curr.pose.position.y = res_translation.y();
    curr.pose.position.z = res_translation.z();

    marker_pose_pub.publish(curr);
}