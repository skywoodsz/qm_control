//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_QMPRECOMPUTATION_H
#define SRC_QMPRECOMPUTATION_H

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "qm_interface/common/ModelSettings.h"
#include "ocs2_legged_robot/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class QMPreComputation : public PreComputation {
public:
    QMPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                              const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
    ~QMPreComputation() override = default;

    QMPreComputation* clone() const override { return new QMPreComputation(*this); }

    void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

    const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

    PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
    const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

protected:
    QMPreComputation(const QMPreComputation& other);

private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
    std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
    const ModelSettings settings_;

    std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

}

#endif //SRC_QMPRECOMPUTATION_H
