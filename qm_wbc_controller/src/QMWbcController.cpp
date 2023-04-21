//
// Created by skywoodsz on 2023/4/21.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "qm_wbc_controller/QMWbcController.h"

#include <qm_estimation/FromTopiceEstimate.h>
#include <qm_wbc/HierarchicalWbc.h>
#include <angles/angles.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_legged_robot/common/utils.h>

#include <pluginlib/class_list_macros.hpp>

namespace qm{
using namespace ocs2;
bool QMWbcController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) {
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

    // ee
    CentroidalModelPinocchioMapping pinocchioMapping(qmInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        qmInterface_->modelSettings().contactNames3DoF);

    std::vector<std::string> eeName{qmInterface_->modelSettings().info.eeFrame};
    armEeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(qmInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                           eeName);

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

    return true;
}

void QMWbcController::setupInterface(const std::string &taskFile, const std::string &urdfFile,
                                     const std::string &referenceFile, bool verbose) {
    qmInterface_ = std::make_shared<QMInterface>(taskFile, urdfFile, referenceFile);
    qmInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void QMWbcController::setupStateEstimate(const std::string &urdfFile,
                                         const std::vector<ContactSensorHandle> &contactSensorHandles,
                                         const hardware_interface::ImuSensorHandle &imuSensorHandle) {
    stateEstimate_ =
            std::make_shared<FromTopicStateEstimate>(qmInterface_->getPinocchioInterface(), qmInterface_->getCentroidalModelInfo(),
                                                     *eeKinematicsPtr_, *armEeKinematicsPtr_, hybridJointHandles_, contactSensorHandles,
                                                     imuSensorHandle);
    currentObservation_.time = 0;
}

QMWbcController::~QMWbcController(){

}

void QMWbcController::starting(const ros::Time &time) {
    currentObservation_.mode = ModeNumber::STANCE;
    const vector_t rbdState = stateEstimate_->update(time, ros::Duration(0.002));

    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(qmInterface_->getPinocchioInterface(),
                                                                      qmInterface_->getCentroidalModelInfo());

    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(rbdState.head(2 *
                                                      qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    currentObservation_.input.setZero(qmInterface_->getCentroidalModelInfo().inputDim);
    ROS_INFO("?");
}

void QMWbcController::stopping(const ros::Time &) {

}

void QMWbcController::update(const ros::Time &time, const ros::Duration &period) {
    // State Estimate
    currentObservation_.time += period.toSec();

    vector_t measuredRbdState = stateEstimate_->update(time, period);
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState.head(2 *
                                                  qmInterface_->getCentroidalModelInfo().generalizedCoordinatesNum));
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();

    // optimal
    size_t plannedMode = 15; // stance
    const auto contactFlags = modeNumber2StanceLeg(plannedMode);
    const vector_t uNominal = weightCompensatingInput(qmInterface_->getCentroidalModelInfo(), contactFlags);

    vector_t optimizedInput(qmInterface_->getCentroidalModelInfo().inputDim);
    vector_t optimizedState(qmInterface_->getCentroidalModelInfo().stateDim);
    optimizedInput.setZero();
    optimizedState.setZero();

    vector_t initState = qmInterface_->getInitialState();
    vector_t armInitState = initState.tail(6);

    optimizedState << currentObservation_.state.head(24), armInitState;
//    optimizedInput = uNominal;

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState, plannedMode,
                              period.toSec(), currentObservation_.time);
    wbcTimer_.endTimer();

    vector_t torque = x.tail(18);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, qmInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, qmInterface_->getCentroidalModelInfo());

    // For init the arm
    if(currentObservation_.time > 15)
    {
        for (size_t j = 0; j < 12; ++j) {
            hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 1, torque(j));
        }
    }

    // arm torque
    for (size_t j = 12; j < 18; ++j) {
        hybridJointHandles_[j].setCommand(posDes(j), 0.0, 0, 0.5, torque(j));
    }
}


}

PLUGINLIB_EXPORT_CLASS(qm::QMWbcController, controller_interface::ControllerBase)