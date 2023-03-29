//
// Created by skywoodsz on 22-12-28.
//
#include "qm_interface/constraint/EndEffectorConstraint.h"
#include "qm_interface/QMPreComputation.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace qm{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const ReferenceManager& referenceManager)
        : StateConstraint(ConstraintOrder::Linear),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          referenceManagerPtr_(&referenceManager) {
    if (endEffectorKinematics.getIds().size() != 1) {
        throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
    }
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
    return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<QMPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

    vector_t constraint(6);
    constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
    constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
    return constraint;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
    // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<QMPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

    auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

    const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
    approximation.dfdx.topRows<3>() = eePosition.dfdx;

    const auto eeOrientationError =
            endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
    approximation.f.tail<3>() = eeOrientationError.f;
    approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

    return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::interpolateEndEffectorPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> {
    const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;

    vector_t position;
    quaternion_t orientation;

    if (stateTrajectory.size() > 1) {
        // Normal interpolation case
        int index;
        scalar_t alpha;
        std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

        const auto& lhs = stateTrajectory[index].tail<7>();
        const auto& rhs = stateTrajectory[index + 1].tail<7>();
        const quaternion_t q_lhs(lhs.tail<4>());
        const quaternion_t q_rhs(rhs.tail<4>());

        position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
        orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
        auto EeState = stateTrajectory.front().tail<7>();
        position = EeState.head<3>();
        orientation = quaternion_t(EeState.tail<4>());
//        position = stateTrajectory.front().head<3>();
//        orientation = quaternion_t(stateTrajectory.front().tail<4>());
    }


    return {position, orientation};
}

}