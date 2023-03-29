//
// Created by skywoodsz on 2023/2/28.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "qm_controllers/QMController.h"
#include "qm_common/math_tool/mathConvert.h"
#include <qm_wbc/HierarchicalWbc.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <angles/angles.h>
#include <qm_estimation/FromTopiceEstimate.h>
#include <qm_estimation/FromArmTopicEstimate.h>

#include <pluginlib/class_list_macros.hpp>

#include <qm_common/debug_tool/dataIo.h>


namespace qm{
using namespace ocs2;
using namespace legged_robot;

bool QMController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) {
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;

    controller_nh.getParam("/urdfFile", urdfFile);
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/referenceFile", referenceFile);

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "qm_interface.verbose", verbose);

    setupInterface(taskFile, urdfFile, referenceFile, verbose);
    setupMpc(controller_nh);
    setupMrt();

    CentroidalModelPinocchioMapping pinocchioMapping(qmInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        qmInterface_->modelSettings().contactNames3DoF);

    std::vector<std::string> eeName{qmInterface_->modelSettings().info.eeFrame};
    armEeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                           eeName);

    // visualize
    ros::NodeHandle nh;
    robotVisualizer_ = std::make_shared<QmVisualizer>(qmInterface_->getPinocchioInterface(),
                                                      qmInterface_->getCentroidalModelInfo(),
                                                      qmInterface_->modelSettings(),
                                                      *eeKinematicsPtr_,
                                                      *armEeKinematicsPtr_,
                                                      nh);

    // Hardware interface
    auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                         "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                         "j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3",
                                         "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"};

    for (const auto& joint_name : joint_names) {
        hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
    }

    auto* contactInterface = robot_hw->get<ContactSensorInterface>();
    std::vector<ContactSensorHandle> contactHandles;
    for (const auto& name : qmInterface_->modelSettings().contactNames3DoF) {
        contactHandles.push_back(contactInterface->getHandle(name));
    }

    // State estimation
    setupStateEstimate(urdfFile, contactHandles, robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));

    // Whole body control
    wbc_ = std::make_shared<HierarchicalWbc>(qmInterface_->getPinocchioInterface(), qmInterface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_, *armEeKinematicsPtr_, controller_nh);
    wbc_->loadTasksSetting(taskFile, true);

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(qmInterface_->getCentroidalModelInfo());


    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh, "K");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_controllers::WeightConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_controllers::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);

    return true;
}

void QMController::starting(const ros::Time &time) {
    // Initial state
    currentObservation_.mode = ModeNumber::STANCE;
    const vector_t rbdState = stateEstimate_->update(time, ros::Duration(0.002));

    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(rbdState.head(2 *
            qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    currentObservation_.input.setZero(qmInterface_->getCentroidalModelInfo().inputDim);

    vector_t EeInitTarget(7), initTarget(qmInterface_->getInitialState().size() + 7);
    EeInitTarget.head(3) << 0.52, 0.09, 0.38 + rbdState[5]; // + 0.056 = 0.436
    EeInitTarget.tail(4) << Eigen::Quaternion<scalar_t>(-0.5, 0.5, -0.5, 0.5).coeffs();


    vector_t initState = qmInterface_->getInitialState();
    vector_t armInitState = initState.tail(6); // 6

    initTarget << currentObservation_.state.head(24), armInitState, EeInitTarget;

    TargetTrajectories target_trajectories({currentObservation_.time}, {initTarget}, {currentObservation_.input});

    std::cout<<"armInitState: "<<armInitState.transpose()<<std::endl;

    // Set the first observation and command and wait for optimization to finish
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    ROS_INFO_STREAM("\033[32m Waiting for the initial policy ... \033[0m");
    while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
        mpcMrtInterface_->advanceMpc();
        ros::WallRate(qmInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
    ROS_INFO_STREAM("\033[32m Initial policy has been received. \033[0m");

    mpcRunning_ = true;
    last_time_ = time;

    optimizedState_ = currentObservation_.state;
}

void QMController::update(const ros::Time &time, const ros::Duration &period) {
    // State Estimate
    currentObservation_.time += period.toSec();

    vector_t measuredRbdState = stateEstimate_->update(time, period);
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState.head(2 *
                                      qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();


    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState, plannedMode, period.toSec(), currentObservation_.time);
    wbcTimer_.endTimer();

    vector_t torque = x.tail(18);

    // id control
//    vector_t joint_acc = vector_t::Zero(qmInterface_->getCentroidalModelInfo().actuatedDofNum);
//    vector_t z = rbdConversions_->computeRbdTorqueFromCentroidalModel(optimizedState, optimizedInput, joint_acc);
//    vector_t torque_id = z.segment<12>(6);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, qmInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, qmInterface_->getCentroidalModelInfo());

     // Safety check, if failed, stop the controller
     if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
         ROS_ERROR_STREAM("[QM Controller] Safety check failed, stopping the controller.");
         stopRequest(time);
     }

    // For init the arm
    if(currentObservation_.time > 10)
    {
        for (size_t j = 0; j < 12; ++j) {
            hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 1, torque(j));
        }
    }

    // arm torque
    for (size_t j = 12; j < 18; ++j) {
        hybridJointHandles_[j].setCommand(posDes(j), 0.0, arm_kp_wbc_, arm_kd_wbc_, torque(j));
    }

    // pos pd control with low frequency
    //    armControl(time, posDes, velDes, torque);

    // TODO: threads
    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

    vector_t ee_state(7); armEeState(ee_state);
    eeStatePublisher_.publish(createEeStateMsg(currentObservation_.time, ee_state));

    SystemObservation currentVelObservation;
    currentVelObservation.time = currentObservation_.time;
    currentVelObservation.state = measuredRbdState.tail(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
    currentVelObservation.input = optimizedInput;
    jointVelPublisher_.publish(ros_msg_conversions::createObservationMsg(currentVelObservation));

    optimizedState_ = optimizedState;
}

void QMController::armControl(const ros::Time &time, ocs2::vector_t posDes, ocs2::vector_t velDes, vector_t torque) {

    if(time - last_time_ > ros::Duration(1.0 / arm_control_loop_hz_)) // arm_control_loop_hz_
    {
        velDes.setZero();
        for(size_t j = 12; j < 12 + 6; ++j){ //
            hybridJointHandles_[j].setCommand(currentObservation_.state(12+j) +
                currentObservation_.input(12+j) * 1.0 / arm_control_loop_hz_
                    , velDes(j), arm_k_[j-12].kp, arm_k_[j-12].kd, 0.0);
        }
        last_time_ = time;
    }
}

qm_msgs::ee_state QMController::createEeStateMsg(ocs2::scalar_t time, ocs2::vector_t state) {
    qm_msgs::ee_state ee_state_msg;

    ee_state_msg.time = time;

    ee_state_msg.state.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        ee_state_msg.state[i] = static_cast<float>(state(i));
    }

    return ee_state_msg;
}

vector_t QMController::invDynamic(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                                  const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period) {
    vector_t qMeasured_, vMeasured_;
    qMeasured_ = vector_t(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
    vMeasured_ = vector_t(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);

    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
    qMeasured_.tail(qmInterface_->getCentroidalModelInfo().actuatedDofNum)
            = rbdStateMeasured.segment(6, qmInterface_->getCentroidalModelInfo().actuatedDofNum);

    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    vMeasured_.tail(qmInterface_->getCentroidalModelInfo().actuatedDofNum)
            = rbdStateMeasured.segment(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum + 6, qmInterface_->getCentroidalModelInfo().actuatedDofNum);

    const auto& model = qmInterface_->getPinocchioInterface().getModel();
    auto& data = qmInterface_->getPinocchioInterface().getData();


    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    vector_t foot_force = -inputDesired.head(12);
    vector_t sol(qmInterface_->getCentroidalModelInfo().actuatedDofNum, 1);

    for (size_t i = 0; i < 4; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, qmInterface_->getCentroidalModelInfo().endEffectorFrameIndices[i],
                                    pinocchio::LOCAL_WORLD_ALIGNED, jac);

        vector_t wrench(6);
        wrench.setZero();
        wrench.head(3) = foot_force.segment(3*i, 3);
        vector_t tau = jac.transpose() * wrench;

        if(i==0)
            sol.segment(0, 3) = tau.segment(6, 3);
        if(i==2)
            sol.segment(3, 3) = tau.segment(6+3, 3);
        if(i==1)
            sol.segment(6, 3) = tau.segment(6+3+3, 3);
        if(i==3)
            sol.segment(9, 3) = tau.segment(6+3+3+3, 3);
    }

    return sol;
}

QMController::~QMController() {
    controllerRunning_ = false;
    if (mpcThread_.joinable()) {
        mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void QMController::setupStateEstimate(const std::string &urdfFile,
                                      const std::vector<ContactSensorHandle> &contactSensorHandles,
                                      const hardware_interface::ImuSensorHandle &imuSensorHandle) {
    stateEstimate_ =
            std::make_shared<FromTopicStateEstimate>(qmInterface_->getPinocchioInterface(), qmInterface_->getCentroidalModelInfo(),
                                                     *eeKinematicsPtr_, *armEeKinematicsPtr_, hybridJointHandles_, contactSensorHandles, imuSensorHandle);
    currentObservation_.time = 0;
}

void QMController::setupMpc(ros::NodeHandle &controller_nh) {
    mpc_ = std::make_shared<SqpMpc>(qmInterface_->mpcSettings(), qmInterface_->sqpSettings(),
                                    qmInterface_->getOptimalControlProblem(), qmInterface_->getInitializer());
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(qmInterface_->getPinocchioInterface(),
                                                                      qmInterface_->getCentroidalModelInfo());

    const std::string robotName = "qm"; // TODO: fix name to qm
    const std::string gaitName = "legged_robot";
    ros::NodeHandle nh;
    // Gait receiver
    auto gaitReceiverPtr =
            std::make_shared<GaitReceiver>(nh, qmInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), gaitName);

    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, qmInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nh);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
    eeStatePublisher_ = nh.advertise<qm_msgs::ee_state>(robotName + "_mpc_observation_ee_state", 1);
    jointVelPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation_joint_vel", 1);
}

void QMController::setupMrt() {
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&qmInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]() {
        while (controllerRunning_) {
            try {
                executeAndSleep(
                        [&]() {
                            if (mpcRunning_) {
                                mpcTimer_.startTimer();
                                mpcMrtInterface_->advanceMpc();
                                mpcTimer_.endTimer();
                            }
                        },
                        qmInterface_->mpcSettings().mpcDesiredFrequency_);
            } catch (const std::exception& e) {
                controllerRunning_ = false;
                ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
            }
        }
    });
    setThreadPriority(qmInterface_->sqpSettings().threadPriority, mpcThread_);
}


void QMController::setupInterface(const std::string &taskFile, const std::string &urdfFile,
                                      const std::string &referenceFile, bool verbose) {
    qmInterface_ = std::make_shared<QMInterface>(taskFile, urdfFile, referenceFile);
    qmInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void QMController::dynamicCallback(qm_controllers::WeightConfig &config, uint32_t) {
    ROS_INFO_STREAM("\033[32m Update the param. \033[0m");
    arm_k_[0].kp = config.kp_arm_1;
    arm_k_[0].kd = config.kd_arm_1;

    arm_k_[1].kp = config.kp_arm_2;
    arm_k_[1].kd = config.kd_arm_2;

    arm_k_[2].kp = config.kp_arm_3;
    arm_k_[2].kd = config.kd_arm_3;

    arm_k_[3].kp = config.kp_arm_4;
    arm_k_[3].kd = config.kd_arm_4;

    arm_k_[4].kp = config.kp_arm_5;
    arm_k_[4].kd = config.kd_arm_5;

    arm_k_[5].kp = config.kp_arm_6;
    arm_k_[5].kd = config.kd_arm_6;

    arm_control_loop_hz_ = config.arm_control_loop_hz;

    arm_kp_wbc_ = config.kp_arm_wbc;
    arm_kd_wbc_ = config.kd_arm_wbc;
}

bool QMControllerDummyObserver::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
// Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;

    controller_nh.getParam("/urdfFile", urdfFile);
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/referenceFile", referenceFile);

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "qm_interface.verbose", verbose);

    setupInterface(taskFile, urdfFile, referenceFile, verbose);
    setupMpc(controller_nh);
    setupMrt();

    CentroidalModelPinocchioMapping pinocchioMapping(qmInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        qmInterface_->modelSettings().contactNames3DoF);

    std::vector<std::string> eeName{qmInterface_->modelSettings().info.eeFrame};
    armEeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                           eeName);

    // visualize
    ros::NodeHandle nh;
    robotVisualizer_ = std::make_shared<QmVisualizer>(qmInterface_->getPinocchioInterface(),
                                                      qmInterface_->getCentroidalModelInfo(),
                                                      qmInterface_->modelSettings(),
                                                      *eeKinematicsPtr_,
                                                      *armEeKinematicsPtr_,
                                                      nh);


    return true;

}

void QMControllerDummyObserver::starting(const ros::Time &time) {
    currentObservation_.mode = ModeNumber::STANCE;
    vector_t rbdState = qmInterface_->getInitialState();
    currentObservation_.state = rbdState;
    currentObservation_.input.setZero(qmInterface_->getCentroidalModelInfo().inputDim);

    vector_t EeInitTarget(7), initTarget(qmInterface_->getInitialState().size() + 7);
    EeInitTarget.head(3) << 0.52, 0.09, 0.38 + rbdState[8];
    EeInitTarget.tail(4) << Eigen::Quaternion<scalar_t>(-0.5, 0.5, -0.5, 0.5).coeffs();

    initTarget << rbdState, EeInitTarget;

    TargetTrajectories target_trajectories({currentObservation_.time}, {initTarget}, {currentObservation_.input});

    // Set the first observation and command and wait for optimization to finish
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    ROS_INFO_STREAM("\033[32m Waiting for the initial policy ... \033[0m");
    while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
        mpcMrtInterface_->advanceMpc();
        ros::WallRate(qmInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
    ROS_INFO_STREAM("\033[32m Initial policy has been received. \033[0m");

    optimizedState_ = rbdState;
    optimizedInput_.setZero(qmInterface_->getCentroidalModelInfo().inputDim);

    mpcRunning_ = true;
}

void QMControllerDummyObserver::update(const ros::Time &time, const ros::Duration &period) {
    // State Estimate
    currentObservation_.time += period.toSec();
    currentObservation_.state = optimizedState_;
    // TODO: mode
    currentObservation_.mode = ModeNumber::STANCE;

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    optimizedState_ = optimizedState;
    optimizedInput_ = optimizedInput;

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    vector_t ee_state(7); armEeState(ee_state);
    eeStatePublisher_.publish(createEeStateMsg(currentObservation_.time, ee_state));
}

void QMController::armEeState(vector_t& ee_state)
{
    armEeKinematicsPtr_->setPinocchioInterface(qmInterface_->getPinocchioInterface());

    vector_t qMeasured_ = centroidal_model::getGeneralizedCoordinates(currentObservation_.state, qmInterface_->getCentroidalModelInfo());

    const auto& model = qmInterface_->getPinocchioInterface().getModel();
    auto& data = qmInterface_->getPinocchioInterface().getData();

    pinocchio::forwardKinematics(model, data, qMeasured_);
    pinocchio::updateFramePlacements(model, data);

    const auto armEePos = armEeKinematicsPtr_->getPosition(vector_t());

    const size_t frame_idx = model.getBodyId(armEeKinematicsPtr_->getIds()[0]);
    const auto armEeOriQuat = matrixToQuaternion(data.oMf[frame_idx].rotation());

    ee_state.segment<4>(3) = vector_t(armEeOriQuat.coeffs());
    ee_state.segment<3>(0) = armEePos[0];
}

bool QMControllerManipulator::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) {
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;

    controller_nh.getParam("/urdfFile", urdfFile);
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/referenceFile", referenceFile);

    bool verbose = false;
    loadData::loadCppDataType(taskFile, "qm_interface.verbose", verbose);

    setupInterface(taskFile, urdfFile, referenceFile, verbose);
    setupMpc(controller_nh);
    setupMrt();
    setHardware(controller_nh);

    CentroidalModelPinocchioMapping pinocchioMapping(qmInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        qmInterface_->modelSettings().contactNames3DoF);

    std::vector<std::string> eeName{qmInterface_->modelSettings().info.eeFrame};
    armEeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(),
                                                                           pinocchioMapping,
                                                                           eeName);

    // visualize
    ros::NodeHandle nh;
    robotVisualizer_ = std::make_shared<QmVisualizer>(qmInterface_->getPinocchioInterface(),
                                                      qmInterface_->getCentroidalModelInfo(),
                                                      qmInterface_->modelSettings(),
                                                      *eeKinematicsPtr_,
                                                      *armEeKinematicsPtr_,
                                                      nh);

    // Hardware interface
    auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                         "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

    for (const auto& joint_name : joint_names) {
        hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
    }

    auto* contactInterface = robot_hw->get<ContactSensorInterface>();
    std::vector<ContactSensorHandle> contactHandles;
    for (const auto& name : qmInterface_->modelSettings().contactNames3DoF) {
        contactHandles.push_back(contactInterface->getHandle(name));
    }

    // State estimation
    setupStateEstimate(urdfFile, contactHandles, robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(qmInterface_->getCentroidalModelInfo());

    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh, "K");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_controllers::WeightConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_controllers::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);

    return true;
}

void QMControllerManipulator::setupStateEstimate(const std::string &urdfFile,
                                                 const std::vector<ContactSensorHandle> &contactSensorHandles,
                                                 const hardware_interface::ImuSensorHandle &imuSensorHandle) {
    stateEstimate_ =
            std::make_shared<FromArmTopicEstimate>(qmInterface_->getPinocchioInterface(), qmInterface_->getCentroidalModelInfo(),
                                                     *eeKinematicsPtr_, *armEeKinematicsPtr_, hybridJointHandles_, contactSensorHandles, imuSensorHandle);
    currentObservation_.time = 0;

}

void QMControllerManipulator::setHardware(ros::NodeHandle& nh) {

    cmd_pub_[0] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_1_position_controller/command", 10);
    cmd_pub_[1] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_2_position_controller/command", 10);
    cmd_pub_[2] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_3_position_controller/command", 10);
    cmd_pub_[3] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_4_position_controller/command", 10);
    cmd_pub_[4] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_5_position_controller/command", 10);
    cmd_pub_[5] =
            std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(nh,
                                                                                   "/joint_6_position_controller/command", 10);
}

void QMControllerManipulator::armControl(const ros::Time &time, ocs2::vector_t posDes, ocs2::vector_t velDes, vector_t torque) {
    if(time - last_time_ > ros::Duration(1.0 / arm_control_loop_hz_)) // arm_control_loop_hz_
    {
        // hardware cmd
        for (size_t i = 0; i < 6; ++i) {
            if(cmd_pub_[i]->trylock())
            {
                cmd_pub_[i]->msg_.data =
                        currentObservation_.state(24+i) + currentObservation_.input(24+i) * 1.0 / arm_control_loop_hz_;
                cmd_pub_[i]->unlockAndPublish();
            }
        }
        last_time_ = time;
    }
}

void QMControllerManipulator::update(const ros::Time &time, const ros::Duration &period) {
    // State Estimate
    currentObservation_.time += period.toSec();

    vector_t measuredRbdState = stateEstimate_->update(time, period);
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState.head(2 *
                                      qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();


    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    // id control
    vector_t joint_acc = vector_t::Zero(qmInterface_->getCentroidalModelInfo().actuatedDofNum);
    vector_t z = rbdConversions_->computeRbdTorqueFromCentroidalModel(optimizedState, optimizedInput, joint_acc);
    vector_t torque_id = z.segment<12>(6);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, qmInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, qmInterface_->getCentroidalModelInfo());

     // Safety check, if failed, stop the controller
     if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
         ROS_ERROR_STREAM("[QM Controller] Safety check failed, stopping the controller.");
         stopRequest(time);
     }


    for (size_t j = 0; j < 12; ++j) {
        hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 1, torque_id(j));
    }
    
    // pos pd control with low frequency
    armControl(time, posDes, velDes, torque_id);

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

    vector_t ee_state(7); armEeState(ee_state);
    eeStatePublisher_.publish(createEeStateMsg(currentObservation_.time, ee_state));

    SystemObservation currentVelObservation;
    currentVelObservation.time = currentObservation_.time;
    currentVelObservation.state = measuredRbdState.tail(qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
    currentVelObservation.input = optimizedInput;
    jointVelPublisher_.publish(ros_msg_conversions::createObservationMsg(currentVelObservation));

    optimizedState_ = optimizedState;
}

} // namespace qm

PLUGINLIB_EXPORT_CLASS(qm::QMController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qm::QMControllerDummyObserver, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(qm::QMControllerManipulator, controller_interface::ControllerBase)