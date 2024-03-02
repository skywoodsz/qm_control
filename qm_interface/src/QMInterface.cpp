//
// Created by skywoodsz on 2023/2/21.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "qm_interface/QMInterface.h"
#include "qm_interface/dynamics/QMDynamicsAD.h"
#include "qm_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "qm_interface/QMPreComputation.h"
#include "qm_interface/initialization/QMInitializer.h"
#include "qm_interface/constraint/EndEffectorConstraint.h"
#include "qm_interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "qm_interface/constraint/EndEffectorForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>

#include <ocs2_legged_robot/constraint/FrictionConeConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroForceConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace qm{
QMInterface::QMInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile)
{

    // check that task file exists
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath)) {
        std::cerr << "[QMInterface] Loading task file: " << taskFilePath << std::endl;
    } else {
        throw std::invalid_argument("[QMInterface] Task file not found: " + taskFilePath.string());
    }

    // check that urdf file exists
    boost::filesystem::path urdfFilePath(urdfFile);
    if (boost::filesystem::exists(urdfFilePath)) {
        std::cerr << "[QMInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
    } else {
        throw std::invalid_argument("[QMInterface] URDF file not found: " + urdfFilePath.string());
    }

    // check that targetCommand file exists
    boost::filesystem::path referenceFilePath(referenceFile);
    if (boost::filesystem::exists(referenceFilePath)) {
        std::cerr << "[QMInterface] Loading target command settings from: " << referenceFilePath << std::endl;
    } else {
        throw std::invalid_argument("[QMInterface] targetCommand file not found: " + referenceFilePath.string());
    }

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "qm_interface.verbose", verbose);

    // load setting from loading file
    modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
    sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
    ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
    rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMInterface::setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile,
                                                 const std::string &referenceFile, bool verbose) {
    setupModel(taskFile, urdfFile, referenceFile, verbose);

    // Initial state
    initialState_.setZero(centroidalModelInfo_.stateDim);
    loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

    setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);

    // Optimal control problem
    problemPtr_ = std::make_unique<OptimalControlProblem>();

    // Dynamics
    std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
    dynamicsPtr = std::make_unique<QMDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_);
    problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

    // Cost terms
    // 1. base & u cost
    problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

    // 2. ee cost
    // TODO: switch for user
    problemPtr_->stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector", verbose));
    problemPtr_->finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector", verbose));

    // Constraint terms
    // 1. joint limits constraint
    problemPtr_->softConstraintPtr->add("armJointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile, verbose));


    // 2. friction cone settings
    scalar_t frictionCoefficient = 0.7;
    RelaxedBarrierPenalty::Config barrierPenaltyConfig;
    std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        const std::string& footName = modelSettings_.contactNames3DoF[i];
        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

        problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                            getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));

        problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(
                *referenceManagerPtr_, i, centroidalModelInfo_)));

        problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i));

        problemPtr_->equalityConstraintPtr->add(
                footName + "_normalVelocity",
                std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, *eeKinematicsPtr, i)));
    }

    // ee force
    Eigen::Vector3d eeForceInit = Eigen::Vector3d::Zero();
    eeForcePtr_ = std::make_shared<SharedValue>(eeForceInit);

    problemPtr_->equalityConstraintPtr->add("ee_wrench", std::unique_ptr<StateInputConstraint>(new EndEffectorForceConstraint(
            centroidalModelInfo_, eeForcePtr_)));


    // Pre-computation
    setupPreComputation(taskFile, urdfFile, referenceFile, verbose);

    // Rollout
    rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

    // Initialization
    constexpr bool extendNormalizedMomentum = true;
    initializerPtr_.reset(new QMInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> QMInterface::getEndEffectorConstraint(const ocs2::PinocchioInterface &pinocchioInterface,
                                                                 const std::string &taskFile, const std::string &prefix, bool verbose) {
    scalar_t muPosition = 1.0;
    scalar_t muOrientation = 1.0;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", verbose);
    loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", verbose);

    if (referenceManagerPtr_ == nullptr) {
        throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
    }

    std::unique_ptr<StateConstraint> constraint;
    const std::string& eeName = modelSettings_.info.eeFrame;
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({eeName}, eeName);
    constraint.reset(new EndEffectorConstraint(*eeKinematicsPtr, *referenceManagerPtr_));

    std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
    std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
    std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

    return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> QMInterface::getJointLimitSoftConstraint(const ocs2::PinocchioInterface &pinocchioInterface,
                                             const std::string &taskFile, bool verbose) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    const auto& model = pinocchioInterface.getModel();

    const int armStateDim = 6;
    const int armInputDim = 6;
    const int armStateIdx = centroidalModelInfo_.stateDim - 6;
    const int armInputIdx = centroidalModelInfo_.inputDim - 6;

    // Load arm position limits
    std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
    {
        scalar_t muPositionLimits = 1e-2;
        scalar_t deltaPositionLimits = 1e-3;

        // arm joint DOF limits from the parsed URDF
        const vector_t lowerBoundArm = model.lowerPositionLimit.tail(armStateDim);
        const vector_t upperBoundArm = model.upperPositionLimit.tail(armStateDim);

        loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", verbose);
        loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", verbose);
        if(verbose)
        {
            std::cerr << "\n #### JointPositionLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### lowerBound: " << lowerBoundArm.transpose() << '\n';
            std::cerr << " #### upperBound: " << upperBoundArm.transpose() << '\n';
            std::cerr << " #### =============================================================================\n";
        }

        stateLimits.reserve(armStateDim);
        for (int i = 0; i < armStateDim; ++i) {
            StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = armStateIdx + i;
            boxConstraint.lowerBound = lowerBoundArm(i);
            boxConstraint.upperBound = upperBoundArm(i);
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
            stateLimits.push_back(std::move(boxConstraint));
        }
    }

    // load arm velocity limits
    std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
    {
        scalar_t muVelocityLimits = 1e-2;
        scalar_t deltaVelocityLimits = 1e-3;

        loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", verbose);
        loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", verbose);

        // arm joint DOFs velocity limits
        vector_t lowerBoundArm = vector_t::Zero(armInputDim);
        vector_t upperBoundArm = vector_t::Zero(armInputDim);
        loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
        loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);

        inputLimits.reserve(armInputDim);
        for (int i = 0; i < armInputDim; ++i) {
            StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = armInputIdx + i;
            boxConstraint.lowerBound = lowerBoundArm(i);
            boxConstraint.upperBound = upperBoundArm(i);
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
            inputLimits.push_back(std::move(boxConstraint));
        }

        if(verbose)
        {
            std::cerr << "\n #### JointVelocityLimits Settings: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << " #### 'lowerBound':  " << lowerBoundArm.transpose() << std::endl;
            std::cerr << " #### 'upperBound':  " << upperBoundArm.transpose() << std::endl;
            std::cerr << " #### =============================================================================\n";
        }
    }

    auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));
    boxConstraints->initializeOffset(0.0, vector_t::Zero(centroidalModelInfo_.stateDim), vector_t::Zero(centroidalModelInfo_.inputDim));
    return boxConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMInterface::setupPreComputation(const std::string &taskFile, const std::string &urdfFile,
                                          const std::string &referenceFile, bool verbose) {
    problemPtr_->preComputationPtr = std::make_unique<QMPreComputation>(
            *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QMInterface::initializeInputCostWeight(const std::string &taskFile, const ocs2::CentroidalModelInfo &info) {
    const size_t totalContactDim = 3 * info.numThreeDofContacts;
    const size_t totalContactDimWithEE = totalContactDim + 6;

    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    const auto q = centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    matrix_t base2feetJac(totalContactDim, info.actuatedDofNum-6);
    for (size_t i = 0; i < info.numThreeDofContacts; i++) {
        matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED, jac);
        base2feetJac.block(3 * i, 0, 3, info.actuatedDofNum-6) = jac.block(0, 6, 3, info.actuatedDofNum-6);
    }

    matrix_t rTaskspace(info.inputDim, info.inputDim);
    loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
    matrix_t r = rTaskspace;

    // Leg Joint velocities
    r.block(totalContactDimWithEE, totalContactDimWithEE, info.actuatedDofNum-6, info.actuatedDofNum-6) =
            base2feetJac.transpose() * rTaskspace.block(totalContactDimWithEE, totalContactDimWithEE, info.actuatedDofNum-6, info.actuatedDofNum-6) *
            base2feetJac;
//    r.block(totalContactDim, totalContactDim, info.actuatedDofNum-6, info.actuatedDofNum-6) =
//            base2feetJac.transpose() * rTaskspace.block(totalContactDim, totalContactDim, info.actuatedDofNum-6, info.actuatedDofNum-6) *
//            base2feetJac;
    return r;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> QMInterface::getBaseTrackingCost(const std::string &taskFile, const ocs2::CentroidalModelInfo &info, bool verbose) {
    matrix_t Q(info.stateDim, info.stateDim);
    loadData::loadEigenMatrix(taskFile, "Q", Q);
    matrix_t R = initializeInputCostWeight(taskFile, info);

    if (verbose) {
        std::cerr << "\n #### Base Tracking Cost Coefficients: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "Q:\n" << Q << "\n";
        std::cerr << "R:\n" << R << "\n";
        std::cerr << " #### =============================================================================\n";
    }

    return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> QMInterface::getZeroVelocityConstraint(const EndEffectorKinematics<ocs2::scalar_t> &eeKinematics,
                                           size_t contactPointIndex)
{
    auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
        EndEffectorLinearConstraint::Config config;
        config.b.setZero(3);
        config.Av.setIdentity(3, 3);
        if (!numerics::almost_eq(positionErrorGain, 0.0)) {
            config.Ax.setZero(3, 3);
            config.Ax(2, 2) = positionErrorGain;
        }
        return config;
    };
    return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                                 eeZeroVelConConfig(modelSettings_.positionErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> QMInterface::getFrictionConeConstraint(size_t contactPointIndex, ocs2::scalar_t frictionCoefficient)
{
    FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
    return std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, frictionConeConConfig, contactPointIndex, centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> QMInterface::getFrictionConeSoftConstraint(size_t contactPointIndex, ocs2::scalar_t frictionCoefficient,
                                               const RelaxedBarrierPenalty::Config &barrierPenaltyConfig)
{
    return std::make_unique<StateInputSoftConstraint>(getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
                                                      std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<EndEffectorKinematics<scalar_t>> QMInterface::getEeKinematicsPtr(const std::vector<std::string> &footNames, const std::string &modelName)
{
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

    const auto infoCppAd = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
    auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
    };
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, footNames,
            centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
    velocityUpdateCallback, modelName, modelSettings_.modelFolderCppAd,
    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    return eeKinematicsPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> QMInterface::loadFrictionConeSettings(const std::string &taskFile, bool verbose) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    const std::string prefix = "frictionConeSoftConstraint.";

    scalar_t frictionCoefficient = 1.0;
    RelaxedBarrierPenalty::Config barrierPenaltyConfig;
    if (verbose) {
        std::cerr << "\n #### Friction Cone Settings: ";
        std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
    if (verbose) {
        std::cerr << " #### =============================================================================\n";
    }

    return {frictionCoefficient, barrierPenaltyConfig};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMInterface::setupModel(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                            bool verbose) {
    pinocchioInterfacePtr_ =
            std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

    centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
            centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
            modelSettings_.contactNames6DoF);

    if (verbose) {
        std::cerr << "\033[32m \n #### pinocchio Info: ";
        std::cerr << "\n #### =============================================================================\n \033[0m";
        std::cerr << "joint name: \n";
        for(auto name : pinocchioInterfacePtr_->getModel().names)
            std::cerr<<name<<"\n";
        std::cerr<<pinocchioInterfacePtr_->getModel().names.size()<<std::endl;
    }


    if (verbose) {
        std::cerr << "\033[32m \n #### centroidal Model Info: ";
        std::cerr << "\n #### =============================================================================\n \033[0m";
        std::cerr << "generalizedCoordinatesNum: "<<centroidalModelInfo_.generalizedCoordinatesNum<<"\n";
        std::cerr << "actuatedDofNum: "<<centroidalModelInfo_.actuatedDofNum<<"\n";
        std::cerr << "stateDim: "<<centroidalModelInfo_.stateDim<<"\n";
        std::cerr << "inputDim: "<<centroidalModelInfo_.inputDim<<"\n";
        std::cerr << "robotMass: "<<centroidalModelInfo_.robotMass<<"\n";
        std::cerr << "qPinocchioNominal: "<<centroidalModelInfo_.qPinocchioNominal.transpose()<<"\n";
    }

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QMInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
    auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
    referenceManagerPtr_ =
            std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> QMInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
    const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
    const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

    const auto defaultGait = [defaultModeSequenceTemplate] {
        Gait gait{};
        gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
        // Events: from time -> phase
        std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                      [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
        // Modes:
        gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
        return gait;
    }();

    // display
    if (verbose) {
        std::cerr << "\n#### Modes Schedule: ";
        std::cerr << "\n#### =============================================================================\n";
        std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
        std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
        std::cerr << "#### =============================================================================\n";
    }

    return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

}