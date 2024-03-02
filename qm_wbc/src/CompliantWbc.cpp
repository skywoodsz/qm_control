//
// Created by skywoodsz on 2023/5/6.
//

#include "qm_wbc/CompliantWbc.h"
#include "qm_wbc/HoQp.h"

namespace qm{
using namespace ocs2;
CompliantWbc::CompliantWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                           const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                           const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                           ros::NodeHandle &controller_nh)
    : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics, controller_nh)
{
    tau_max_.setZero();
    imp_desired_.resize(generalizedCoordinatesNum_);
    imp_desired_.setZero();

    // init compliant controller
    impendace_controller_ = std::make_shared<CartesianImpendance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    BoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseBoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);

    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"compliant");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_wbc::CompliantConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}
// init bounded admittance of joints
void CompliantWbc::BoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    bounded_admittance_controller2_ = std::make_shared<BoundedAdmittance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller2_->setJointIdx(2);

    bounded_admittance_controller1_ = std::make_shared<BoundedAdmittance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller1_->setJointIdx(1);
}
// init bounded admittance of the base
void CompliantWbc::BaseBoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                             const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    bounded_admittance_controller_base_x_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller_base_x_->setJointIdx(0);

//    bounded_admittance_controller_base_y_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
//    bounded_admittance_controller_base_y_->setJointIdx(1);
}

// bounded admittance of joints control law
vector6_t CompliantWbc::BoundedAdmittanceUpdate(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time,
                                                ocs2::scalar_t period, ocs2::vector_t imp) {
    vector6_t tau_feedback, tau_desired, tau_addmitance;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_addmitance.setZero();
    tau_feedback = rbdStateMeasured.tail(6);
    tau_desired = imp.tail(6);

    vector_t tmp;
    bounded_admittance_controller2_->setTorqueMax(tau_max_[2]);
    bounded_admittance_controller2_->setParam(Mb2_, Kb2_, Bb2_, Lb2_, Mbx_, Bbx_);
    bounded_admittance_controller2_->setForceFeedback(tau_feedback[2]);
    bounded_admittance_controller2_->setForceDesired(tau_desired[2]);
    tmp = bounded_admittance_controller2_->update(rbdStateMeasured, time, period);
    tau_addmitance[2] = tmp[1];

    bounded_admittance_controller1_->setTorqueMax(tau_max_[1]);
    bounded_admittance_controller1_->setParam(Mb1_, Kb1_, Bb1_, Lb1_, Mbx_, Bbx_);
    bounded_admittance_controller1_->setForceFeedback(tau_feedback[1]);
    bounded_admittance_controller1_->setForceDesired(tau_desired[1]);
    tmp = bounded_admittance_controller1_->update(rbdStateMeasured, time, period);
    tau_addmitance[1] = tmp[1];

    return tau_addmitance;
}
// bounded admittance of base control law
vector6_t CompliantWbc::BaseBoundedAdmittanceUpdate(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time,
                                                    ocs2::scalar_t period, ocs2::vector_t imp, scalar_t force_z) {
    vector_t tau_desired(2), tau_addmitance(2);
    tau_desired.setZero();
    tau_desired = imp.head(2);

    // get the desired position from the reference
    scalar_t base_xd{};
    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;
        const size_t size = timeTrajectory.size();
        base_xd = stateTrajectory[size-1][6];
    }

    vector_t tmp;
    bounded_admittance_controller_base_x_->setTorqueMax(force_z * mu_); // frictionCoeff_ * force_z
    bounded_admittance_controller_base_x_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
    bounded_admittance_controller_base_x_->setDesired(0, 0, base_xd);
//    bounded_admittance_controller_base_x_->setDesired(baseAccDesired_[0], vDesired_[0], qDesired_[0]);
    bounded_admittance_controller_base_x_->setForceDesired(tau_desired[0]);
    bounded_admittance_controller_base_x_->setForceFeedback(0);
    tmp = bounded_admittance_controller_base_x_->update(rbdStateMeasured, time, period);
    tau_addmitance[0] = tmp[1];

//    bounded_admittance_controller_base_y_->setTorqueMax(force_z * mu_); // mpc force z * mu
//    bounded_admittance_controller_base_y_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
//    bounded_admittance_controller_base_y_->setDesired(0, 0, base_y_);
//    bounded_admittance_controller_base_y_->setForceDesired(tau_desired[1]);
//    bounded_admittance_controller_base_y_->setForceFeedback(0);
//    tmp = bounded_admittance_controller_base_y_->update(rbdStateMeasured, time, period);
//    tau_addmitance[1] = tmp[1];

    return tau_addmitance;
}
// bounded admittance control law
vector_t CompliantWbc::BoundanceAdmittanceControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
             const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period, ocs2::scalar_t time) {
    // get desire pose and velocity from planner
    vector3_t eeDesiredPosition = WbcBase::getEEPosition();
    vector3_t eeDesiredVelocity = WbcBase::getEEVelocity();

    // impendance control
    impendace_controller_->setDesiredPosition(eeDesiredPosition);
    impendace_controller_->setDesiredVelocity(eeDesiredVelocity);
    vector_t imp = impendace_controller_->update(rbdStateMeasured, time, period);
    imp_desired_ = imp;

    // arm's bounded admittance
    vector_t ba_tau = BoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp);
    vector6_t compliant_tau = imp.tail(6);
    for (int i = 1; i < 3; ++i) {
        compliant_tau[i] = ba_tau[i];
    }

    // base's bounded admittance
    scalar_t force_z = 0;
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    }
    vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t proxy_x = bounded_admittance_controller_base_x_->getProxyState();

    // constraint
    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                 + formulateNoContactMotionTask() + formulateFrictionConeTask();
    // compliance
    Task taskBA = formulateManipulatorTorqueTask(compliant_tau.tail(6));
    Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);

    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                 + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy + formulateBaseYLinearMotionTask();

//    Task task3 = formulateContactForceTask(inputDesired);
    Task task3 = formulateZContactForceTask(inputDesired) + formulateYContactForceTask(inputDesired)
            + formulateXContactForceTaskWithCompliant(inputDesired, tau_cmd);

    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();
    return WbcBase::updateCmdWithEEForce(x_optimal);
}

vector_t CompliantWbc::update(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                              const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period,
                              ocs2::scalar_t time) {
    // wbc
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    setManipulatorTorqueLimit(tau_max_);

    if(time < 5)
    {
        Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                     + formulateNoContactMotionTask() + formulateFrictionConeTask();
        Task taskInit = formulateArmJointNomalTrackingTask();
        Task task3 = formulateContactForceTask(inputDesired);

        HoQp hoQp(task3, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
        vector_t x_optimal = hoQp.getSolutions();
        return WbcBase::updateCmdWithEEForce(x_optimal);
    }

    return BoundanceAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
}

void CompliantWbc::dynamicCallback(qm_wbc::CompliantConfig &config, uint32_t) {
    tau_max_ << config.joint1_tau_max, config.joint2_tau_max, config.joint3_tau_max,
            config.joint4_tau_max, config.joint5_tau_max, config.joint6_tau_max;

    // bounde admittance for arm
    Mbx_ = config.Mbx;
    Bbx_ = config.Bbx;
    Mb2_ = config.Mb2;
    Kb2_ = config.Kb2;
    Bb2_ = config.Bb2;
    Lb2_ = config.Lb2;
    Mb1_ = config.Mb1;
    Kb1_ = config.Kb1;
    Bb1_ = config.Bb1;
    Lb1_ = config.Lb1;

    // bounde admittance for base
    Mbase_px_ = config.Mbase_px;
    Bbase_px_ = config.Bbase_px;
    Kbase_px_ = config.Kbase_px;
    Mbase_py_ = config.Mbase_py;
    Bbase_py_ = config.Bbase_py;
    Kbase_py_ = config.Kbase_py;

    Mbasex_ = config.Mbasex;
    Bbasex_ = config.Bbasex;
    Kbasex_ = config.Kbasex;
    Lbasex_ = config.Lbasex;
    Mbasey_ = config.Mbasey;
    Bbasey_ = config.Bbasey;
    Kbasey_ = config.Kbasey;
    Lbasey_ = config.Lbasey;

    mu_ = config.mu;

    ROS_INFO_STREAM("\033[32m Update the CompliantWbc param. \033[0m");
}

}
