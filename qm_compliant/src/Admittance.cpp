//
// Created by skywoodsz on 2023/4/25.
//

#include "qm_compliant/Admittance.h"
#include "qm_compliant/Numerics.h"

namespace qm{
Admittance::Admittance(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                       const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
        : CompliantBase(pinocchioInterface, info, armEeKinematics, controller_nh)
{
    initParam();
}

void Admittance::initParam() {
    ddot_qx_ = 0.;
    dot_qx_ = 0.;
    qx_ = 0.;
    a_ = 0.;
    tau_ = 0.;
    dot_q0_ = 0.;
    q0_ = 0.;
    T_ = 0.;

    init_flag_ = false;
    idx_flag_ = false;
    param_flag_ = false;
}

void Admittance::setJointIdx(size_t idx) {
    idx_ = idx;
    idx_flag_ = true;
}

void Admittance::setParam(ocs2::scalar_t M, ocs2::scalar_t K, ocs2::scalar_t B, ocs2::scalar_t L, ocs2::scalar_t Mx,
                          ocs2::scalar_t Bx, ocs2::scalar_t Kx) {
    M_ = M;
    K_ = K;
    B_ = B;
    L_ = L;

    Mx_ = Mx;
    Bx_ = Bx;
    Kx_ = Kx;

    param_flag_ = true;
}

scalar_t Admittance::projectionFunction(ocs2::scalar_t x) {
    if(almost_ge(x, tau_max_))
        x = tau_max_;
    if(almost_le(x, -tau_max_))
        x = -tau_max_;
        
    return x;
}

vector_t Admittance::getProxyState() {
    vector_t state(3); // qx, qx_dot, qx_ddot

    state(0) = qx_;
    state(1) = dot_qx_;
    state(2) = ddot_qx_;

    return state;
}

vector_t Admittance::update(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time, ocs2::scalar_t period) {
//    CompliantBase::update(rbdStateMeasured, time, period);

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
        qx_pub_= nh.advertise<std_msgs::Float64>("/admittance/qs" + id_string, 1);
        f_pub_= nh.advertise<std_msgs::Float64>("/admittance/f_error" + id_string, 1);
        tau_pub_ = nh.advertise<std_msgs::Float64>("/admittance/tau_cmd" + id_string, 1);


        qx_ = joint_pos;
        dot_qx_ = joint_vel;
    }

    // control law
    T_ = period;

    ddot_qx_ = 1 / Mx_ * (-Bx_ * dot_qx_ - Kx_ * (qx_ - q0_) + fd_ - f_);
    dot_qx_ = dot_qx_ + T_ * ddot_qx_;
    qx_ = qx_ + T_ * dot_qx_;
    a_ = a_ + T_ * (qx_ - joint_pos);
    tau_ = M_ * ddot_qx_ + K_ * (qx_ - joint_pos) + B_ * (dot_qx_ - joint_vel) + L_ * a_;

    vector_t tau(1);
    tau << projectionFunction(tau_);

    // publish msg
    std_msgs::Float64 msg;
    msg.data = qx_;
    qx_pub_.publish(msg);

    std_msgs::Float64 f_msg;
    f_msg.data = fd_ - f_;
    f_pub_.publish(f_msg);

    std_msgs::Float64 cmd_msg;
    cmd_msg.data = tau[0];
    tau_pub_.publish(cmd_msg);

    return tau;
}




}
