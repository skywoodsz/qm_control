//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_QMROBOTDYNAMICSAD_H
#define SRC_QMROBOTDYNAMICSAD_H

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "qm_interface/common/ModelSettings.h"

namespace qm{
using namespace ocs2;

class QMDynamicsAD final : public SystemDynamicsBase{
public:
    QMDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info, const std::string& modelName,
                 const ModelSettings& modelSettings);

    ~QMDynamicsAD() override = default;

    QMDynamicsAD* clone() const override {return new QMDynamicsAD(*this);}

    vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
    VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                          const PreComputation& preComp) override;

private:
    QMDynamicsAD(const QMDynamicsAD& rhs) = default;

    PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;
};

}


#endif //SRC_QMROBOTDYNAMICSAD_H
