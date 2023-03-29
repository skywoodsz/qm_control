//
// Created by skywoodsz on 2023/2/28.
//

#ifndef SRC_QMINITIALIZER_H
#define SRC_QMINITIALIZER_H

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class QMInitializer final : public Initializer{
public:
    QMInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                  bool extendNormalizedMomentum = false);
    ~QMInitializer() override = default;
    QMInitializer* clone() const override;

    void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
    QMInitializer(const QMInitializer& other) = default;
    const CentroidalModelInfo info_;
    const SwitchedModelReferenceManager* referenceManagerPtr_;
    const bool extendNormalizedMomentum_;
};
}


#endif //SRC_QMINITIALIZER_H
