//
// Created by skywoodsz on 2023/4/9.
//

#ifndef SRC_HIERARCHICALWBC_H
#define SRC_HIERARCHICALWBC_H

#include "qm_wbc/WbcBase.h"

namespace qm {

class HierarchicalWbc : public WbcBase {
public:
    HierarchicalWbc(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                    const PinocchioEndEffectorKinematics &eeKinematics,
                    const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                    ros::NodeHandle &controller_nh);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode,
                    scalar_t period, scalar_t time) override;
};
}
#endif //SRC_HIERARCHICALWBC_H
