//
// Created by skywoodsz on 2023/4/23.
//

#include "qm_compliant/BoundedAdmittance.h"
#include "qm_compliant/Numerics.h"

namespace qm{
using namespace ocs2;
BoundedAdmittance::BoundedAdmittance(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                     const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
            : CompliantBase(pinocchioInterface, info, armEeKinematics, controller_nh)
{
    initParam();
}

void BoundedAdmittance::initParam() {
    Mx_ = 0.;
    Bx_ = 0.;
    Kx_ = 0.;

    f_ = 0.;
    fd_ = 0.;

    M_ = 0.;
    B_ = 0.;
    K_ = 0.;
    L_ = 0.;

    ux_star_ = 0.;
    qx_start_ = 0.;
    phi_b_ = 0.;
    phi_a_ = 0.;
    qs_star_ = 0.;
    tau_star_ = 0.;
    tau_ = 0.;
    qx_ = 0.;
    ux_ = 0.;
    a_ = 0.;
    qs_ = 0.;

    ux_star_pre_ = 0.;
    qx_start_pre_ = 0.;
    phi_b_pre_ = 0.;
    phi_a_pre_ = 0.;
    qs_star_pre_ = 0.;
    tau_star_pre_ = 0.;
    tau_pre_ = 0.;
    qx_pre_ = 0.;
    ux_pre_ = 0.;
    a_pre_ = 0.;
    qs_pre_ = 0.;

    T_ = 0.;

    init_flag_ = false;
    idx_flag_ = false;
    param_flag_ = false;
}

void BoundedAdmittance::setParam(scalar_t M, scalar_t K, scalar_t B, scalar_t L, scalar_t Mx, scalar_t Bx) {
    M_ = M;
    K_ = K;
    B_ = B;
    L_ = L;

    Mx_ = Mx;
    Bx_ = Bx;

    param_flag_ = true;
}

void BoundedAdmittance::setJointIdx(size_t idx) {
    idx_ = idx;
    idx_flag_ = true;
}

scalar_t BoundedAdmittance::projectionFunction(scalar_t x) {
    if(almost_ge(x, tau_max_))
        x = tau_max_;
    if(almost_le(x, -tau_max_))
        x = -tau_max_;
    return x;
}

vector_t BoundedAdmittance::update(const vector_t &rbdStateMeasured, scalar_t time, scalar_t period) {
    if(!idx_flag_ || !param_flag_)
    {
        ROS_INFO("Need set joint idx of param first!");
        return {};
    }

    vector_t arm_joint_pos = rbdStateMeasured.segment(generalizedCoordinatesNum_-6, 6);
    vector_t arm_joint_vel = rbdStateMeasured.segment(2*generalizedCoordinatesNum_-6, 6);
    scalar_t joint_pos = arm_joint_pos(idx_);
    scalar_t joint_vel = arm_joint_vel(idx_);

    if(!init_flag_)
    {
        init_flag_ = true;
        ros::NodeHandle nh;
        std::string id_string = std::to_string(idx_);
        qx_pub_= nh.advertise<std_msgs::Float64>("/BoundedAdmittance/qs" + id_string, 1);
        f_pub_= nh.advertise<std_msgs::Float64>("/BoundedAdmittance/f_error" + id_string, 1);
        tau_pub_ = nh.advertise<std_msgs::Float64>("/BoundedAdmittance/cmd" + id_string, 1);

        qx_ = joint_pos;
        qx_pre_ = joint_pos;
        qs_ = joint_pos;
        qs_pre_ = joint_pos;

        ux_ = joint_vel;
        ux_pre_ = joint_vel;
    }

    qs_ = joint_pos;
    T_ = period;

    // Bounded
    // control law
    scalar_t Khat_ = K_ + B_/T_ + L_*T_;
    ux_star_ = (Mx_ * ux_pre_ + T_ * (fd_ - f_)) / (Mx_ + Bx_ * T_); // TODO: convert
    qx_start_ = qx_pre_ + T_ * ux_star_;
    phi_b_ = B_ * (qx_pre_ - qs_pre_) / T_ - L_ * a_pre_;
    phi_a_ = M_ * (qs_ - qx_pre_ - T_ * ux_pre_) / (T_ * T_);
    qs_star_ = qs_ + (phi_b_ - phi_a_) / (Khat_ + M_/(T_ * T_));
    tau_star_ = (Khat_ + M_/(T_ * T_)) * (qx_start_ - qs_star_);
    tau_ = projectionFunction(tau_star_); // problem of 精度
    qx_ = qs_star_ + tau_ / (Khat_ + M_/(T_ * T_));
    ux_ = (qx_ - qx_pre_) / T_;
    a_ = a_pre_ + T_ * (qx_ - qs_);

    // update
    ux_pre_ = ux_;
    qx_pre_ = qx_;
    qs_pre_ = qs_;
    a_pre_ = a_;

    vector_t tau_cmd(2);
    tau_cmd.setZero();
    tau_cmd[1] = tau_;
    tau_cmd[0] = tau_pre_;

    tau_pre_ = tau_;

    // publish msg
    std_msgs::Float64 msg;
    msg.data = qx_;
    qx_pub_.publish(msg);

    std_msgs::Float64 f_msg;
    f_msg.data = fd_ - f_;
    f_pub_.publish(f_msg);

    std_msgs::Float64 tau_msg;
    tau_msg.data = tau_cmd[1];
    tau_pub_.publish(tau_msg);

    return tau_cmd;
}

}
