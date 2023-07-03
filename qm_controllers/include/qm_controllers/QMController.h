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

struct K
{
    scalar_t kp;
    scalar_t kd;
};


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
    virtual void setupInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                bool verbose);
    virtual void setupMpc(ros::NodeHandle& controller_nh);
    virtual void setupMrt();
    virtual void setupStateEstimate(const std::string& urdfFile, const std::vector<ContactSensorHandle>& contactSensorHandles,
                                    const hardware_interface::ImuSensorHandle& imuSensorHandle);
    virtual void armControl(const ros::Time &time, vector_t posDes, vector_t velDes, vector_t torque);

    vector_t invDynamic(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                        scalar_t period);
    qm_msgs::ee_state createEeStateMsg(scalar_t time, vector_t state);
    void armEeState(vector_t& ee_state);
    void dynamicCallback(qm_controllers::WeightConfig& config, uint32_t /*level*/);

    std::shared_ptr<QMInterface> qmInterface_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> armEeKinematicsPtr_;

    std::shared_ptr<MPC_BASE> mpc_;
    std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    ros::Publisher observationPublisher_, eeStatePublisher_, jointVelPublisher_;

    SystemObservation currentObservation_;
    std::vector<HybridJointHandle> hybridJointHandles_;

    std::shared_ptr<SafetyChecker> safetyChecker_;

    std::thread mpcThread_;
    std::atomic_bool controllerRunning_{}, mpcRunning_{};
    benchmark::RepeatedTimer mpcTimer_;
    benchmark::RepeatedTimer wbcTimer_;

    std::shared_ptr<dynamic_reconfigure::Server<qm_controllers::WeightConfig>> dynamic_srv_{};
    K arm_k_[6];
    double arm_control_loop_hz_;
    ros::Time last_time_;

    std::shared_ptr<QmVisualizer> robotVisualizer_;
    std::shared_ptr<WbcBase> wbc_;
    
    double arm_kp_wbc_{}, arm_kd_wbc_{};
    
private:
    vector_t optimizedState_;
    
};


/**
 * qm controller with arm manipulator
 */
class QMControllerManipulator : public QMController{
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
private:
    void setupStateEstimate(const std::string& urdfFile, const std::vector<ContactSensorHandle>& contactSensorHandles,
                                 const hardware_interface::ImuSensorHandle& imuSensorHandle) override;
    void setHardware(ros::NodeHandle& nh);
    void armControl(const ros::Time &time, vector_t posDes, vector_t velDes, vector_t torque) override;

    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> cmd_pub_[6];
    realtime_tools::RealtimeBuffer<sensor_msgs::JointState> joint_state_buffer_;
    vector_t optimizedState_;
};

}


#endif //SRC_QMCONTROLLER_H
