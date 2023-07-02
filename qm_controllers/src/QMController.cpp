//
// Created by skywoodsz on 2023/2/28.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "qm_controllers/QMController.h"
#include <qm_wbc/HierarchicalWbc.h>
#include <qm_estimation/LinearKalmanFilter.h>

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

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

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
    for (const auto& name : qmInterface_->modelSettings().contactNames3DoF) {
        contactHandles_.push_back(contactInterface->getHandle(name));
    }
    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");


    // State estimation
    setupStateEstimate(taskFile, verbose);

    // Whole body control
    wbc_ = std::make_shared<HierarchicalWbc>(qmInterface_->getPinocchioInterface(), qmInterface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_, *armEeKinematicsPtr_, controller_nh);
    wbc_->loadTasksSetting(taskFile, true);

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(qmInterface_->getCentroidalModelInfo());

    // Intergrator
    arm_qdot_intergrator_.SetSize(6);
    arm_qdot_intergrator_.Reset();
    vector_t arm_qdot_limit = vector_t::Zero(6);
    arm_qdot_limit << 0.628, 0.628, 0.628, 0.837, 0.837, 0.837;
    arm_qdot_intergrator_.SetLimit(arm_qdot_limit);

    dog_control_ = false;
    arm_control_ = false;
    // Dynamic reconfigure
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
    currentObservation_.state.setZero(qmInterface_->getCentroidalModelInfo().stateDim);
    updateStateEstimation(time, ros::Duration(0.002));
    currentObservation_.input.setZero(qmInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;

    // Initial target
    vector_t EeInitTarget(7), initTarget(qmInterface_->getInitialState().size() + 7);
    EeInitTarget.head(3) << 0.52, 0.09, 0.38 + measuredRbdState_[5]; // + 0.056 = 0.436
    EeInitTarget.tail(4) << Eigen::Quaternion<scalar_t>(-0.5, 0.5, -0.5, 0.5).coeffs();
    vector_t initState = qmInterface_->getInitialState();
    vector_t armInitState = initState.tail(6);
    initTarget << currentObservation_.state.head(24), armInitState, EeInitTarget;
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

    mpcRunning_ = true;
}

void QMController::update(const ros::Time &time, const ros::Duration &period) {
    // State Estimate
    updateStateEstimation(time, period);

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
    currentObservation_.input = optimizedInput;

    // Whole body control
    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec(), currentObservation_.time);
    wbcTimer_.endTimer();
    vector_t torque = x.tail(18);
    vector_t q_ddot = x.segment<6>(18);

//    arm_qdot_intergrator_.Integrate(q_ddot, period.toSec());
//    vector_t arm_q_dot = vector_t::Zero(6);
//    arm_q_dot = arm_qdot_intergrator_.GetIntegral();

    // id control
    // vector_t joint_acc = vector_t::Zero(qmInterface_->getCentroidalModelInfo().actuatedDofNum);
    // vector_t z = rbdConversions_->computeRbdTorqueFromCentroidalModel(optimizedState, optimizedInput, joint_acc);
    // vector_t torque_id = z.segment<12>(6);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, qmInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, qmInterface_->getCentroidalModelInfo());

     // Safety check, if failed, stop the controller
     if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
         ROS_ERROR_STREAM("[QM Controller] Safety check failed, stopping the controller.");
         stopRequest(time);
     }

    //  Dog control
    if(dog_control_)
    {
        for (size_t j = 0; j < 12; ++j) {
            hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
        }
    }


    // Arm torque
    if(arm_control_)
    {
        for (size_t j = 12; j < 18; ++j) { 
            hybridJointHandles_[j].setCommand(posDes(j), velDes(j), arm_kp_wbc_, arm_kd_wbc_, 0.0);
        }
    }

    // TODO: threads
    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

    vector_t ee_state = measuredRbdState_.tail(7);
    eeStatePublisher_.publish(createEeStateMsg(currentObservation_.time, ee_state));
}

void QMController::updateStateEstimation(const ros::Time &time, const ros::Duration &period) {
    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

    // Joint state
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
        jointPos(i) = hybridJointHandles_[i].getPosition();
        jointVel(i) = hybridJointHandles_[i].getVelocity();
    }

    // Contact state
    for (size_t i = 0; i < contacts.size(); ++i) {
        contactFlag[i] = contactHandles_[i].isContact();
    }
    // IMU state
    for (size_t i = 0; i < 4; ++i) {
        quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
    }
    for (size_t i = 0; i < 3; ++i) {
        angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
        linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
    }

    for (size_t i = 0; i < 9; ++i) {
        orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
        angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
        linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
    }

    // Update
    stateEstimate_->updateJointStates(jointPos, jointVel);
    stateEstimate_->updateContact(contactFlag);
    stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
    measuredRbdState_ = stateEstimate_->update(time, period); // state + ee

    currentObservation_.time += period.toSec();
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_.head(2 *
                                               qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum)); // no ee state
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();
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

void QMController::setupStateEstimate(const std::string& taskFile, bool verbose) {
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(qmInterface_->getPinocchioInterface(),
                                                            qmInterface_->getCentroidalModelInfo(),
                                                            *eeKinematicsPtr_, *armEeKinematicsPtr_);

    currentObservation_.time = 0;
}

void QMController::setupMpc(ros::NodeHandle &controller_nh) {
    mpc_ = std::make_shared<SqpMpc>(qmInterface_->mpcSettings(), qmInterface_->sqpSettings(),
                                    qmInterface_->getOptimalControlProblem(), qmInterface_->getInitializer());
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(qmInterface_->getPinocchioInterface(),
                                                                      qmInterface_->getCentroidalModelInfo());

    const std::string robotName = "qm"; 
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

void QMController::dynamicCallback(qm_controllers::WeightConfig &config, uint32_t) {
    ROS_INFO_STREAM("\033[32m Update the param. \033[0m");

    arm_kp_wbc_ = config.kp_arm_wbc;
    arm_kd_wbc_ = config.kd_arm_wbc;

    dog_control_ = config.dog_control;
    arm_control_ = config.arm_control;
}

} // namespace qm

PLUGINLIB_EXPORT_CLASS(qm::QMController, controller_interface::ControllerBase)