//
// Created by skywoodsz on 2023/5/12.
//

#ifndef SRC_QMHW_H
#define SRC_QMHW_H

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <qm_common/hardware_interface/ContactSensorInterface.h>
#include <qm_common/hardware_interface/HybridJointInterface.h>
#include <realtime_tools/realtime_publisher.h>
#include <qm_msgs/MotorState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

// Unitree
#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/safety.h"

// Kinova
#include <kinova_driver/kinova_comm.h>


namespace qm{
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct EeForceData{
    double x_, y_, z_;
};

struct QmMotorData {
    double pos_, vel_, tau_, temperature_;                 // state
    double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
    double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
    double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
    double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
    double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
    double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
    double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class QmHW : public hardware_interface::RobotHW{
public:
    QmHW() = default;

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;
    void stop();

private:
    bool loadUrdf(ros::NodeHandle& rootNh);
    bool setupJoints();
    bool setupImu();
    bool setupContactSensor(ros::NodeHandle& nh);
    void UnitreeStateRead();
    void KinovaStateRead();
    void UnitreeCmdWrite();
    void KinovaCmdWrite();
    void publishState(const ros::Time& time);
    double convertKinDeg(const double& qd);
    bool KinovaInit();
    bool temperatureProtection();

    // Interface
    hardware_interface::JointStateInterface jointStateInterface_;
    hardware_interface::ImuSensorInterface imuSensorInterface_;
    HybridJointInterface hybridJointInterface_;
    ContactSensorInterface contactSensorInterface_;

    // URDF model of the robot
    std::shared_ptr<urdf::Model> urdfModel_;

    // Unitree wrapper
    std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
    std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
    UNITREE_LEGGED_SDK::LowState lowState_{};
    UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

    // Kinova wrapper
    std::shared_ptr<kinova::KinovaComm> kinova_comm_;
    std::vector<std::string> arm_joint_names_;

    QmMotorData jointData_[18]{};
    UnitreeImuData imuData_{};
    EeForceData eeForce_;
    float l_joint_torque_[COMMAND_SIZE];

    size_t dogIndex_ = 12;
    bool contactState_[4]{};
    int powerLimit_{};
    int contactThreshold_{};

    // Time
    ros::Time last_publish_time_;
    ros::Time last_kinova_cmd_time_;
    ros::Time last_kinova_state_time_;
    std::shared_ptr<realtime_tools::RealtimePublisher<qm_msgs::MotorState>> motorStatePub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imuStatePub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> wrenchPub_;
};

}




#endif //SRC_QMHW_H
