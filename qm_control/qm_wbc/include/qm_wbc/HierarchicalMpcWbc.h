//
// Created by skywoodsz on 2023/7/1.
//

#ifndef SRC_HIERARCHICALMPCWBC_H
#define SRC_HIERARCHICALMPCWBC_H

#include "qm_wbc/WbcBase.h"

namespace qm {

class HierarchicalMpcWbc : public WbcBase {
public:
    HierarchicalMpcWbc(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics &eeKinematics,
                        const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                        ros::NodeHandle &controller_nh);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode,
                    scalar_t period, scalar_t time) override;
};
}

#endif //SRC_HIERARCHICALMPCWBC_H
