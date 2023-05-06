//
// Created by skywoodsz on 2023/5/5.
//

#ifndef SRC_ENDEFFECTORFORCECONSTRAINT_H
#define SRC_ENDEFFECTORFORCECONSTRAINT_H

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <memory>

#include "qm_interface/common/SharedValue.h"

namespace qm{
using namespace ocs2;
class EndEffectorForceConstraint final : public StateInputConstraint{
public:
    EndEffectorForceConstraint(CentroidalModelInfo info, std::shared_ptr<SharedValue> eeForcePtr);
    ~EndEffectorForceConstraint() override = default;
    EndEffectorForceConstraint* clone() const override { return new EndEffectorForceConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override { return 6; }
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                             const PreComputation& preComp) const override;

private:
    EndEffectorForceConstraint(const EndEffectorForceConstraint& other) = default;

    const CentroidalModelInfo info_;
    std::shared_ptr<SharedValue> eeForcePtr_;
};

}

#endif //SRC_ENDEFFECTORFORCECONSTRAINT_H
