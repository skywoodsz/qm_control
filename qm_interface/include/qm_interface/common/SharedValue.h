//
// Created by skywoodsz on 2023/5/6.
//

#ifndef SRC_SHAREDVALUE_H
#define SRC_SHAREDVALUE_H

#include <ocs2_core/Types.h>
#include "ocs2_core/thread_support/BufferedValue.h"
#include "ocs2_oc/synchronized_module/SolverSynchronizedModule.h"

namespace qm{
using namespace ocs2;

class SharedValue : public SolverSynchronizedModule{
public:
    SharedValue(Eigen::Vector3d initValue);

    void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                      const ReferenceManagerInterface& referenceManager) override;
    void postSolverRun(const PrimalSolution& primalSolution) override{}

    void setSharedValue(const Eigen::Vector3d value){ value_.setBuffer(value); }
    const Eigen::Vector3d& getSharedValue(){ return value_.get(); }

private:
    BufferedValue<Eigen::Vector3d> value_;

};

}


#endif //SRC_SHAREDVALUE_H
