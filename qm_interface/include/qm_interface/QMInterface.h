//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_LEGGEDINTERFACE_H
#define SRC_LEGGEDINTERFACE_H

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

#include "qm_interface/common/ModelSettings.h"
#include "qm_interface/common/SharedValue.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class QMInterface : public RobotInterface {
public:
    QMInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);
    ~QMInterface() override = default;

    virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose);

    const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

    const ModelSettings& modelSettings() const { return modelSettings_; }
    const ddp::Settings& ddpSettings() const { return ddpSettings_; }
    const mpc::Settings& mpcSettings() const { return mpcSettings_; }
    const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
    const sqp::Settings& sqpSettings() { return sqpSettings_; }
    const ipm::Settings& ipmSettings() { return ipmSettings_; }

    const vector_t& getInitialState() const { return initialState_; }
    const RolloutBase& getRollout() const { return *rolloutPtr_; }
    PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
    const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
    PinocchioGeometryInterface& getGeometryInterface() { return *geometryInterfacePtr_; }
    std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

    const Initializer& getInitializer() const override { return *initializerPtr_; }
    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

    std::shared_ptr<SharedValue> getEeForcePtr() const { return eeForcePtr_; }

protected:
    virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);
    virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                       bool verbose);
    virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);

    std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

    std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
    matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

    static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose);
    std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
    std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                  const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
    std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& footNames,
                                                                        const std::string& modelName);
    std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex);
    std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                const std::string& taskFile, bool verbose);
    std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& prefix, bool verbose);

    ModelSettings modelSettings_;
    mpc::Settings mpcSettings_;
    ddp::Settings ddpSettings_;
    sqp::Settings sqpSettings_;
    ipm::Settings ipmSettings_;

    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    CentroidalModelInfo centroidalModelInfo_;
    std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

    std::unique_ptr<OptimalControlProblem> problemPtr_;
    std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

    rollout::Settings rolloutSettings_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> initializerPtr_;

    std::shared_ptr<SharedValue> eeForcePtr_;

    vector_t initialState_;
};

}



#endif //SRC_LEGGEDINTERFACE_H
