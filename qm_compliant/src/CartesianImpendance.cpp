//
// Created by skywoodsz on 2023/4/22.
//

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "qm_compliant/CartesianImpendance.h"

namespace qm{
using namespace ocs2;
CartesianImpendance::CartesianImpendance(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
    : CompliantBase(pinocchioInterface, info, armEeKinematics, controller_nh),
      pinocchioInterfaceImpendance_(pinocchioInterface)
{
    D_ = matrix_t(3, 3);
    K_ = matrix_t(3, 3);
    Dr_ = matrix_t(3, 3);
    Kr_ = matrix_t(3, 3);
    vd_.setZero();

    desired_init_flag_ = false;

    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"impendance");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_compliant::ImpemdanceConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_compliant::ImpemdanceConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}

void CartesianImpendance::dynamicCallback(qm_compliant::ImpemdanceConfig &config, uint32_t) {
    vector3_t xd;
    xd.setZero();
    D_.setZero();
    K_.setZero();
    Dr_.setZero();
    Kr_.setZero();
    rd_.setZero();


    D_.diagonal() << config.d, config.d, config.d;
    K_.diagonal() << config.k, config.k, config.k;

    Kr_.diagonal() << config.kr, config.kr, config.kr;
    Dr_.diagonal() << config.dr, config.dr, config.dr;

    xd << config.x_x, config.x_y, config.x_z;

    // debug
//    setDesiredPosition(xd);
    rd_ << config.r_z, config.r_y, config.r_x;

    ROS_INFO_STREAM("\033[32m Update the Cartesian Impendance param. \033[0m");
}

void CartesianImpendance::setDesiredPosition(vector3_t xd) {
    xd_ = xd;
    desired_init_flag_ = true;
}

// TODO: desired with mpc
vector_t CartesianImpendance::update(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time, ocs2::scalar_t period) {
    CompliantBase::update(rbdStateMeasured, time, period);

    // cmd
    return calculateCommandedTorques(period);
}

vector_t CartesianImpendance::calculateCommandedTorques(ocs2::scalar_t period) {

    if(!desired_init_flag_)
    {
        ROS_INFO("Need set desired position first");
        return {};
    }

    size_t generalizedCoordinatesNum = getGeneralizedCoordinatesNum();
    const auto& model = pinocchioInterfaceImpendance_.getModel();
    auto& data = pinocchioInterfaceImpendance_.getData();

    // fext
    vector6_t f_ext = vector6_t::Zero();
    vector3_t err_pos, err_vel;
    err_pos = xd_ - x_;
    err_vel = vd_ - v_;
    f_ext.segment<3>(0) = D_ * err_vel + K_ * err_pos;

    vector3_t errorRotationPos = CartesianImpendance::getOrientationError(rd_);
    vector3_t errorRotationVel = -w_;
    f_ext.segment<3>(3) = Dr_ * errorRotationVel + Kr_ * errorRotationPos;
//    f_ext.segment<3>(3) = Dr_ * errorRotationVel;

    // desired
    vector_t ddot_qd = vector_t(generalizedCoordinatesNum);
    vector_t dot_qd = vector_t(generalizedCoordinatesNum);
    ddot_qd.setZero();
    dot_qd.setZero();

    // tau
    pinocchio::forwardKinematics(model, data, qMeasured_, dot_qd);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::nonLinearEffects(model, data, qMeasured_, dot_qd);

    vector_t tau = vector_t(generalizedCoordinatesNum);
    tau = data.nle + arm_j_.transpose() * f_ext;

    return tau;
}

}

