//
// Created by skywoodsz on 2023/2/21.
//
// copy from: https://github.com/qiayuanliao/legged_control

#include "qm_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "qm_interface/QMPreComputation.h"

namespace qm {
using namespace ocs2;
using namespace legged_robot;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(
    const SwitchedModelReferenceManager &referenceManager,
    const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
    size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd &rhs)
        : StateInputConstraint(rhs),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t
NormalVelocityConstraintCppAd::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                        const PreComputation &preComp) const {
    const auto &preCompLegged = cast<QMPreComputation>(preComp);
    eeLinearConstraintPtr_->configure(
            preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

    return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation
NormalVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t &state,
                                                      const vector_t &input,
                                                      const PreComputation &preComp) const {
    const auto &preCompLegged = cast<QMPreComputation>(preComp);
    eeLinearConstraintPtr_->configure(
            preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

    return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}


}
