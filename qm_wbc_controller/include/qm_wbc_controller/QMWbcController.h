//
// Created by skywoodsz on 2023/4/21.
//

#ifndef SRC_QMWBCCONTROLLER_H
#define SRC_QMWBCCONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <qm_common/hardware_interface/HybridJointInterface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <qm_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include <qm_estimation/StateEstimateBase.h>
#include <qm_interface/QMInterface.h>
#include <qm_interface/visualization/qm_visualization.h>
#include <qm_wbc/WbcBase.h>

#include <qm_msgs/ee_state.h>
#include <qm_msgs/arm_torque.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace qm{
class QMWbcController : public controller_interface::MultiInterfaceController<HybridJointInterface,
        hardware_interface::ImuSensorInterface, ContactSensorInterface>{
public:
    QMWbcController() = default;
    ~QMWbcController() override;

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& /*time*/) override;
protected:
    void setupInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                bool verbose);
    void setupStateEstimate(const std::string& urdfFile, const std::vector<ContactSensorHandle>& contactSensorHandles,
                                    const hardware_interface::ImuSensorHandle& imuSensorHandle);

    std::shared_ptr<QMInterface> qmInterface_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> armEeKinematicsPtr_;

    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;

    SystemObservation currentObservation_;
    std::vector<HybridJointHandle> hybridJointHandles_;

    benchmark::RepeatedTimer wbcTimer_;

    std::shared_ptr<WbcBase> wbc_;
};

}


#endif //SRC_QMWBCCONTROLLER_H
