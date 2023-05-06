//
// Created by skywoodsz on 2023/2/28.
//

#include "qm_interface/initialization/QMInitializer.h"

#include "ocs2_legged_robot/initialization/LeggedRobotInitializer.h"
#include "ocs2_legged_robot/common/utils.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QMInitializer::QMInitializer(ocs2::CentroidalModelInfo info,
                             const ocs2::legged_robot::SwitchedModelReferenceManager &referenceManager,
                             bool extendNormalizedMomentum)
 : info_(std::move(info)), referenceManagerPtr_(&referenceManager), extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QMInitializer *QMInitializer::clone() const {
        return new QMInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMInitializer::compute(ocs2::scalar_t time, const ocs2::vector_t &state, ocs2::scalar_t nextTime,
                                ocs2::vector_t &input, ocs2::vector_t &nextState) {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    input = weightCompensatingInput(info_, contactFlags);
    nextState = state;
    if (!extendNormalizedMomentum_) {
        centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
    }
}
}
