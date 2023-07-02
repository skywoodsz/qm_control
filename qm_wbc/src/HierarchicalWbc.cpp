//
// Created by skywoodsz on 2023/4/9.
//

#include "qm_wbc/HierarchicalWbc.h"

#include "qm_wbc/HoQp.h"

namespace qm{
HierarchicalWbc::HierarchicalWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const ocs2::PinocchioEndEffectorKinematics& armEeKinematics,
                                 ros::NodeHandle &controller_nh)
        : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics, controller_nh){
}

vector_t HierarchicalWbc::update(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                                 const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period,
                                 ocs2::scalar_t time) {

    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
             + formulateFrictionConeTask() + formulateNoContactMotionTask();

    Task task1 = formulateBaseHeightMotionTask() + formulateBaseLinearTask()
           + formulateSwingLegTask() * 100 + formulateBaseAngularMotionTask();

    Task task2 = formulateContactForceTask(inputDesired);

    HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

    vector_t x_optimal = hoQp.getSolutions();
    return WbcBase::updateCmd(x_optimal);
}


}