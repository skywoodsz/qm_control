//
// Created by skywoodsz on 2023/5/3.
//
#include "qm_compliant/BoundedAdmittanceWithK.h"
#include "qm_compliant/Numerics.h"

namespace qm{
using namespace ocs2;

BoundedAdmittanceWithK::BoundedAdmittanceWithK(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                               const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    generalizedCoordinatesNum_ = info.generalizedCoordinatesNum;
    initParam();
}

void BoundedAdmittanceWithK::initParam() {
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
    ax_ = 0.;

    qx_pre_ = 0.;
    ux_pre_ = 0.;
    a_pre_ = 0.;
    qs_pre_ = 0.;
    tau_pre_ = 0.;

    T_ = 0.;

    init_flag_ = false;
    idx_flag_ = false;
    param_flag_ = false;
    tau_flag_ = false;
}

void BoundedAdmittanceWithK::setJointIdx(size_t idx) {
    idx_ = idx;
    idx_flag_ = true;
}

void BoundedAdmittanceWithK::setTorqueMax(ocs2::scalar_t tau_max) {
    tau_max_ = tau_max;
    tau_flag_ = true;
}

void BoundedAdmittanceWithK::setParam(ocs2::scalar_t M, ocs2::scalar_t K, ocs2::scalar_t B, ocs2::scalar_t L,
                                      ocs2::scalar_t Mx, ocs2::scalar_t Bx, ocs2::scalar_t Kx) {
    M_ = M;
    K_ = K;
    B_ = B;
    L_ = L;

    Mx_ = Mx;
    Bx_ = Bx;
    Kx_ = Kx;

    param_flag_ = true;
}

void BoundedAdmittanceWithK::setDesired(ocs2::scalar_t ddot_q0, ocs2::scalar_t dot_q0, ocs2::scalar_t q0) {
    ddot_q0_ = ddot_q0;
    dot_q0_ = dot_q0;
    q0_ = q0;
}

scalar_t BoundedAdmittanceWithK::projectionFunction(ocs2::scalar_t x) {
    if(almost_ge(x, tau_max_))
        x = tau_max_;
    if(almost_le(x, -tau_max_))
        x = -tau_max_;
    return x;
}

vector_t BoundedAdmittanceWithK::getProxyState() {
    vector_t state(3); // qx, qx_dot, qx_ddot

    state(0) = qx_;
    state(1) = ux_;
    state(2) = ax_;

    return state;
}

vector_t BoundedAdmittanceWithK::update(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time, ocs2::scalar_t period) {
    if(!idx_flag_ || !param_flag_ || !tau_flag_)
    {
        ROS_INFO("Need set joint idx of param first!");
        return {};
    }

    scalar_t base_pos = rbdStateMeasured(3 + idx_);
    scalar_t base_vel = rbdStateMeasured(generalizedCoordinatesNum_ + 3 +idx_);

    if(!init_flag_)
    {
        init_flag_ = true;
        ros::NodeHandle nh;

        std::string id_string = std::to_string(idx_);
        qx_pub_ = nh.advertise<std_msgs::Float64>("/BoundedAdmittance/base/qs" + id_string, 1);
        f_pub_ = nh.advertise<std_msgs::Float64>("/BoundedAdmittance/base/force_limit" + id_string, 1);
        tau_pub_ = nh.advertise<std_msgs::Float64>("/BoundedAdmittance/base/force" + id_string, 1);

        qx_ = base_pos;
        qx_pre_ = base_pos;
        qs_ = base_pos;
        qs_pre_ = base_pos;
        ux_ = base_vel;
        ux_pre_ = base_vel;
    }

    T_ = period;
    qs_ = base_pos;

    // control law
    scalar_t Khat_ = K_ + B_/T_ + L_*T_;
    scalar_t Mat1 = 1 + Kx_ * T_ * T_ / (Mx_ + Bx_ * T_);
    scalar_t Mat2 = Khat_ + M_ / (T_ * T_);
    ux_star_ = (Mx_ * ux_pre_ + T_ * (Mx_ * ddot_q0_ +Bx_ * dot_q0_ + Kx_ * q0_ - fd_ + f_)) / (Mx_ + Bx_ * T_);
    qx_start_ = qx_pre_ + T_ * ux_star_;
    phi_b_ = B_ * (qx_pre_ - qs_pre_) / T_ - L_ * a_pre_;
    phi_a_ = M_ * (qs_ - qx_pre_ - T_ * ux_pre_) / (T_ * T_);
    qs_star_ = qs_ + (phi_b_ - phi_a_) / (Khat_ + M_/(T_ * T_));
    tau_star_ = Mat2 / Mat1 * qx_start_ - Mat2 * qs_star_;
    tau_ = projectionFunction(tau_star_);
    qx_ = qs_star_ + tau_ / (Khat_ + M_/(T_ * T_));
    ux_ = (qx_ - qx_pre_) / T_;
    a_ = a_pre_ + T_ * (qx_ - qs_);
    ax_ = (ux_ - ux_pre_) / T_;

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
    f_msg.data = tau_max_;
    f_pub_.publish(f_msg);

    std_msgs::Float64 tau_msg;
    tau_msg.data = tau_;
    tau_pub_.publish(tau_msg);

    return tau_cmd;
}


}

