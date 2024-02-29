//
// Created by skywoodsz on 2023/2/21.
//

#include "qm_interface/dynamics/QMDynamicsAD.h"

namespace qm {
    using namespace ocs2;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QMDynamicsAD::QMDynamicsAD(const PinocchioInterface &pinocchioInterface, const CentroidalModelInfo &info,
                           const std::string &modelName,
                           const ModelSettings &modelSettings)
        : pinocchioCentroidalDynamicsAd_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd,
                                         modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd) {}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QMDynamicsAD::computeFlowMap(scalar_t time, const vector_t &state, const vector_t &input,
                                      const PreComputation &preComp) {
    return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation QMDynamicsAD::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                             const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
}
}