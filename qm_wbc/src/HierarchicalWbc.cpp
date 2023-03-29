//
// Created by skywoodsz on 2023/3/13.
//
// copy from:

#include "qm_wbc/HierarchicalWbc.h"

#include "qm_wbc/HoQp.h"

namespace qm{
HierarchicalWbc::HierarchicalWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const ocs2::PinocchioEndEffectorKinematics& armEeKinematics,
                                 ros::NodeHandle &controller_nh)
        : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics,controller_nh){
}


vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

    Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
                 + formulateNoContactMotionTask() + formulateFrictionConeTask();
    Task taskInit = formulateArmJointTrackingTask();
    Task task1 = formulateBaseHeightMotionTask() + formulateEeLinearMotionTrackingTask()
            + formulateBaseAngularMotion() + formulateSwingLegTask() * 100;
    Task task2 = formulateArmJointTrackingTask() + formulateBaseXYLinearAccelTask();
    Task task3 = formulateContactForceTask(inputDesired);

    if(time < 10)
    {
        HoQp hoQp(task3, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
        return hoQp.getSolutions();
    }
    else
    {
        HoQp hoQp(task3, std::make_shared<HoQp>(task2 ,std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))));
        return hoQp.getSolutions();
    }
}
}