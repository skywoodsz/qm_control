//
// Created by skywoodsz on 2023/7/1.
//

#include "qm_wbc/HierarchicalMpcWbc.h"

#include "qm_wbc/HoQp.h"

namespace qm{
HierarchicalMpcWbc::HierarchicalMpcWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const ocs2::PinocchioEndEffectorKinematics& armEeKinematics,
                                 ros::NodeHandle &controller_nh)
        : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics, controller_nh){
}


vector_t HierarchicalMpcWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {

    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

    Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
                 + formulateNoContactMotionTask() + formulateFrictionConeTask();

    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask()
        + formulateBaseLinearMotionTask() + formulateSwingLegTask() * 100;

    Task task2 = formulateContactForceTask(inputDesired);

    HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();
    return WbcBase::updateCmd(x_optimal);
}
}