//
// Created by skywoodsz on 2023/3/3.
//

#ifndef SRC_LEGGEDROBOTQUADRATICTRACKINGCOST_H
#define SRC_LEGGEDROBOTQUADRATICTRACKINGCOST_H

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "ocs2_legged_robot/common/utils.h"
#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

/**
 * State-input tracking cost used for intermediate times
 */
class LeggedRobotStateInputQuadraticCost final : public QuadraticStateInputCost {
public:
    LeggedRobotStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                       const SwitchedModelReferenceManager& referenceManager)
            : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

    ~LeggedRobotStateInputQuadraticCost() override = default;
    LeggedRobotStateInputQuadraticCost* clone() const override { return new LeggedRobotStateInputQuadraticCost(*this); }

private:
    LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs) = default;

    std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                         const TargetTrajectories& targetTrajectories) const override {
        const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
        const vector_t xNominal = targetTrajectories.getDesiredState(time).head(30); // no ee
        const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
        return {state - xNominal, input - uNominal};
    }

    const CentroidalModelInfo info_;
    const SwitchedModelReferenceManager* referenceManagerPtr_;
};

/**
 * State tracking cost used for the final time
 */
class LeggedRobotStateQuadraticCost final : public QuadraticStateCost {
public:
    LeggedRobotStateQuadraticCost(matrix_t Q, CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
            : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

    ~LeggedRobotStateQuadraticCost() override = default;
    LeggedRobotStateQuadraticCost* clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

private:
    LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost& rhs) = default;

    vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
        const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
        const vector_t xNominal = targetTrajectories.getDesiredState(time).head(30);
        return state - xNominal;
    }

    const CentroidalModelInfo info_;
    const SwitchedModelReferenceManager* referenceManagerPtr_;
};

}



#endif //SRC_LEGGEDROBOTQUADRATICTRACKINGCOST_H
