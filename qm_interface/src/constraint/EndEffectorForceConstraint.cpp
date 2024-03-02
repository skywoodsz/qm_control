//
// Created by skywoodsz on 2023/5/5.
//

#include "qm_interface/constraint/EndEffectorForceConstraint.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace qm{
using namespace ocs2;

EndEffectorForceConstraint::EndEffectorForceConstraint(ocs2::CentroidalModelInfo info, std::shared_ptr<SharedValue> eeForcePtr)
    : StateInputConstraint(ConstraintOrder::Linear),
      info_(std::move(info)),
      eeForcePtr_(std::move(eeForcePtr)) {}


bool EndEffectorForceConstraint::isActive(ocs2::scalar_t time) const {
    bool flag  = false;
    if(info_.numSixDofContacts > 0)
        flag = true;

    return flag;
}

vector_t EndEffectorForceConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                         const ocs2::PreComputation &preComp) const {
    vector_t ee_wrench = vector_t::Zero(6);
    vector_t eeForce = vector_t::Zero(3);

    eeForce = eeForcePtr_->getSharedValue();
    ee_wrench.head(3) = eeForce;

    return - ee_wrench + input.segment<6>(3 * info_.numThreeDofContacts);
}

VectorFunctionLinearApproximation EndEffectorForceConstraint::getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                   const ocs2::vector_t &input,
                                                   const ocs2::PreComputation &preComp) const {
    VectorFunctionLinearApproximation approx;
    approx.f = getValue(time, state, input, preComp);
    approx.dfdx = matrix_t::Zero(6, state.size());
    approx.dfdu = matrix_t::Zero(6, input.size());
    approx.dfdu.middleCols<6>(3 * info_.numThreeDofContacts).diagonal() = vector_t::Ones(6);
    return approx;
}

}
