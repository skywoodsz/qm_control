//
// Created by skywoodsz on 2023/2/21.
//


#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include "qm_interface/QMPreComputation.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

QMPreComputation::QMPreComputation(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                   const ocs2::legged_robot::SwingTrajectoryPlanner &swingTrajectoryPlanner,
                                   ModelSettings settings)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
          mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
          settings_(std::move(settings))
{
    eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QMPreComputation::QMPreComputation(const QMPreComputation& rhs)
        : pinocchioInterface_(rhs.pinocchioInterface_),
          info_(rhs.info_),
          swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
          mappingPtr_(rhs.mappingPtr_->clone()),
          settings_(rhs.settings_) {
    eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
    if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
        return;
    }

    // lambda to set config for normal velocity constraints
    auto eeNormalVelConConfig = [&](size_t footIndex) {
        EndEffectorLinearConstraint::Config config;
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
            config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
            config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
        }
        return config;
    };

    if (request.contains(Request::Constraint)) {
        for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
            eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
        }
    }

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    vector_t q = mappingPtr_->getPinocchioJointPosition(x);
    if (request.contains(Request::Approximation)) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::updateGlobalPlacements(model, data);
        pinocchio::computeJointJacobians(model, data);

        updateCentroidalDynamics(pinocchioInterface_, info_, q);
        vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
        updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
    } else {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    }
}
}


