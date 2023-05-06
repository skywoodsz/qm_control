//
// Created by skywoodsz on 2023/5/6.
//

#include "qm_interface/common/SharedValue.h"

namespace qm{

SharedValue::SharedValue(Eigen::Vector3d initValue)
    : value_(std::move(initValue)) {}

void SharedValue::preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t &currentState,
                              const ocs2::ReferenceManagerInterface &referenceManager) {
    value_.updateFromBuffer();
}

}
