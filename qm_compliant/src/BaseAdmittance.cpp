//
// Created by skywoodsz on 2023/5/1.
//

#include "qm_compliant/BaseAdmittance.h"

namespace qm{
BaseAdmittance::BaseAdmittance(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                               const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
    : Admittance(pinocchioInterface, info, armEeKinematics, controller_nh)
{
    
}

vector_t BaseAdmittance::update(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time, ocs2::scalar_t period) {
    if(!idx_flag_ || !param_flag_)
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
        qx_pub_= nh.advertise<std_msgs::Float64>("/admittance/base/qs" + id_string, 1);
        f_pub_ = nh.advertise<std_msgs::Float64>("/admittance/base/force_limit" + id_string, 1);
        tau_pub_ = nh.advertise<std_msgs::Float64>("/admittance/base/force" + id_string, 1);

        qx_ = base_pos;
        dot_qx_ = base_vel;
    }

    T_ = period;

    ddot_qx_ = 1 / Mx_ * (-Bx_ * dot_qx_ - Kx_ * (qx_ - q0_) - fd_);
    dot_qx_ = dot_qx_ + T_ * ddot_qx_;
    qx_ = qx_ + T_ * dot_qx_;
    a_ = a_ + T_ * (qx_ - base_pos);
    tau_ = M_ * ddot_qx_ + K_ * (qx_ - base_pos) + B_ * (dot_qx_ - base_vel) + L_ * a_;

    vector_t tau(1);
    tau << projectionFunction(tau_);

    // publish msg
    std_msgs::Float64 msg;
    msg.data = qx_;
    qx_pub_.publish(msg);

    std_msgs::Float64 f_msg;
    f_msg.data = tau_max_;
    f_pub_.publish(f_msg);

    std_msgs::Float64 tau_msg;
    tau_msg.data = tau[0];
    tau_pub_.publish(tau_msg);

    return tau;
}


}
