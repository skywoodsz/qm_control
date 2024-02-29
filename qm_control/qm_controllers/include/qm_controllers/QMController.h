//
// Created by skywoodsz on 2023/2/28.
//

#ifndef SRC_QMCONTROLLER_H
#define SRC_QMCONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <qm_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include <qm_estimation/StateEstimateBase.h>
#include <qm_interface/QMInterface.h>
#include <qm_interface/visualization/qm_visualization.h>
#include <qm_wbc/WbcBase.h>
#include "qm_controllers/SafetyChecker.h"

#include <dynamic_reconfigure/server.h>
#include "qm_controllers/WeightConfig.h"

#include <qm_msgs/ee_state.h>
#include <qm_msgs/arm_torque.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;

class QMController : public controller_interface::MultiInterfaceController<HybridJointInterface,
        hardware_interface::ImuSensorInterface, ContactSensorInterface> {
public:
    QMController() = default;
    ~QMController() override;

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

protected:
    virtual void setHybridJointHardware(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);
    virtual void setupInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                bool verbose);
    virtual void setupMpc(ros::NodeHandle& controller_nh);
    virtual void setupMrt();
    virtual void setupWbc(ros::NodeHandle& controller_nh, const std::string& taskFile);
    virtual void setupStateEstimate(const std::string& taskFile, bool verbose);
    virtual void updateJointState(vector_t& jointPos, vector_t& jointVel);
    virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
    virtual void updateControlLaw(const vector_t &posDes, const vector_t &velDes, const vector_t &torque);

    qm_msgs::ee_state createEeStateMsg(scalar_t time, vector_t state);
    void dynamicCallback(qm_controllers::WeightConfig& config, uint32_t /*level*/);

    // Interface
    std::shared_ptr<QMInterface> qmInterface_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> armEeKinematicsPtr_;
    std::vector<ContactSensorHandle> contactHandles_;
    std::vector<HybridJointHandle> hybridJointHandles_;
    hardware_interface::ImuSensorHandle imuSensorHandle_;

    // State Estimation
    SystemObservation currentObservation_;
    vector_t measuredRbdState_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

    // MPC & WBC
    std::shared_ptr<MPC_BASE> mpc_;
    std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;
    std::shared_ptr<WbcBase> wbc_;
    std::shared_ptr<SafetyChecker> safetyChecker_;
    std::thread mpcThread_;
    std::atomic_bool controllerRunning_{}, mpcRunning_{};
    benchmark::RepeatedTimer mpcTimer_;
    benchmark::RepeatedTimer wbcTimer_;

    // Visualization
    std::shared_ptr<QmVisualizer> robotVisualizer_;
    ros::Publisher observationPublisher_, eeStatePublisher_;

    // Debug
    std::shared_ptr<dynamic_reconfigure::Server<qm_controllers::WeightConfig>> dynamic_srv_{};
    scalar_t arm_kp_wbc_{}, arm_kd_wbc_{};
    scalar_t last_time_{};
};

class QMMpcController : public QMController{
public:
    QMMpcController() = default;
    ~QMMpcController() = default;

private:
    void setHybridJointHardware(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void setupWbc(ros::NodeHandle& controller_nh, const std::string& taskFile);
    void updateJointState(vector_t& jointPos, vector_t& jointVel);
    void updateControlLaw(const vector_t &posDes, const vector_t &velDes, const vector_t &torque);

    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> cmd_pub_[6];
    ros::Subscriber arm_joint_sub_;
    realtime_tools::RealtimeBuffer<sensor_msgs::JointState> joint_state_buffer_;
};

}




#endif //SRC_QMCONTROLLER_H