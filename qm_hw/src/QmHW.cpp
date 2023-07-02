//
// Created by skywoodsz on 2023/5/12.
//

#include "qm_hw/QmHW.h"

#include <kinova_driver/kinova_ros_types.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/CartesianForce.h>
#include <kinova_msgs/ArmJointAnglesAction.h>

namespace qm{
double QmHW::convertKinDeg(const double &qd) {
    static const double PI_180 = (M_PI / 180.0);
    double qdr = qd;
    if (qd > 180.0) {
        qdr = qd - 360.0;
    }
    qdr *= PI_180 / 2.0; // velocity feedback is 2 times for real value

    return qdr;
}

bool QmHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    // Load URDF
    if (!loadUrdf(root_nh)){
        ROS_ERROR("Error occurred while setting up urdf");
        return false;
    }

    // Register Interface
    registerInterface(&jointStateInterface_);
    registerInterface(&hybridJointInterface_);
    registerInterface(&imuSensorInterface_);
    registerInterface(&contactSensorInterface_);

    robot_hw_nh.getParam("power_limit", powerLimit_);

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    // Unitree Dog
    udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
    udp_->InitCmdData(lowCmd_);
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);

    // Kinova Arm
    boost::recursive_mutex api_mutex;
    bool is_first_init = true;
    std::string kinova_robotType = "j2n6s300";
    kinova_comm_ = std::make_shared<kinova::KinovaComm>(root_nh, api_mutex, is_first_init,kinova_robotType);
    // Init for Kinova
    KinovaInit();

    readTimer_.reset();
    writeTimer_.reset();

    imuStatePub_.reset(
            new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(root_nh, "/imu_data", 1));
    motorStatePub_.reset(
            new realtime_tools::RealtimePublisher<qm_msgs::MotorState>(root_nh, "/motor_states", 1));
    wrenchPub_.reset(
            new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(root_nh, "/ee_force", 1));

    ROS_INFO("\033[1;32m [QmHW]: Hardware start successfully! \033[0m");
    return true;
}

bool QmHW::temperatureProtection() {
    for (size_t i = 0; i < 12; ++i) {
        if(jointData_[i].temperature_ > 75)
        {
            return true;
        }
    }
    return false;
}

void QmHW::stop() {
    kinova_comm_->SetTorqueControlState(0);

    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto& name : names) {
        HybridJointHandle handle = hybridJointInterface_.getHandle(name);
        handle.setFeedforward(0.);
        handle.setVelocityDesired(0.);
        handle.setKd(3.);
    }

    std::cerr << "########################################################################";
    std::cerr << "\n### Read Benchmarking";
    std::cerr << "\n###   Maximum : " << readTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << readTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### Write Benchmarking";
    std::cerr << "\n###   Maximum : " << writeTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << writeTimer_.getAverageInMilliseconds() << "[ms].";
}

bool QmHW::KinovaInit() {
    // init pose
    float init_angle[6] = {3.14, 3.61, 0.86, 2.7, 1.37, -0.4}; // rad
    bool action_is_over = false;
    float tolerance = 2.0;
    kinova::KinovaAngles current_joint_angles, target_joint_angles;

    kinova_comm_->getJointAngles(current_joint_angles);
    target_joint_angles.Actuator1 = init_angle[0] * 180.0 / M_PI;
    target_joint_angles.Actuator2 = init_angle[1] * 180.0 / M_PI;
    target_joint_angles.Actuator3 = init_angle[2] * 180.0 / M_PI;
    target_joint_angles.Actuator4 = init_angle[3] * 180.0 / M_PI;
    target_joint_angles.Actuator5 = init_angle[4] * 180.0 / M_PI;
    target_joint_angles.Actuator6 = init_angle[5] * 180.0 / M_PI;

    ROS_INFO("\033[1;32m [QmHW]: Arm begin to init... \033[0m");
    kinova_comm_->setJointAngles(target_joint_angles);
    while(!action_is_over)
    {
        kinova_comm_->getJointAngles(current_joint_angles);
        if(target_joint_angles.isCloseToOther(current_joint_angles, tolerance))
            action_is_over = true;
        ros::WallRate(20.0).sleep();
    }
    ROS_INFO("\033[1;32m [QmHW]: Arm init successfully! \033[0m");

    // init torque
    for (size_t i = 0; i < COMMAND_SIZE; i++)
    {
        l_joint_torque_[i] = 0;
    }

    // switch to torque control model
//     kinova_comm_->SetTorqueControlState(1);

    return true;
}

void QmHW::UnitreeStateRead() {
    udp_->Recv();
    udp_->GetRecv(lowState_);

    for (int i = 0; i < 12; ++i) {
        jointData_[i].pos_ = lowState_.motorState[i].q;
        jointData_[i].vel_ = lowState_.motorState[i].dq;
        jointData_[i].tau_ = lowState_.motorState[i].tauEst;
        jointData_[i].temperature_ = lowState_.motorState[i].temperature;
    }

    imuData_.ori_[0] = lowState_.imu.quaternion[1];
    imuData_.ori_[1] = lowState_.imu.quaternion[2];
    imuData_.ori_[2] = lowState_.imu.quaternion[3];
    imuData_.ori_[3] = lowState_.imu.quaternion[0];
    imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
    imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
    imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
    imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
    imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
    imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactState_[i] = lowState_.footForce[i] > contactThreshold_;
    }
}

void QmHW::KinovaStateRead() {
    // Joint angles
    kinova::KinovaAngles current_angles;
    kinova_comm_->getJointAngles(current_angles);
    kinova_msgs::JointAngles kinova_angles = current_angles.constructAnglesMsg();

    // Joint velocities
    kinova::KinovaAngles current_vels;
    kinova_comm_->getJointVelocities(current_vels);

    // Joint torques
    kinova::KinovaAngles joint_tqs;
    kinova_comm_->getJointTorques(joint_tqs); // torque with gravity

    // End-effort force
    // TODO: ee force interface
//    kinova::KinovaPose wrench;
//    kinova_comm_->getCartesianForce(wrench);
//    eeForce_.x_ = wrench.X;
//    eeForce_.y_ = wrench.Y;
//    eeForce_.z_ = wrench.Z;

    jointData_[dogIndex_+0].pos_ = kinova_angles.joint1 * M_PI/180;
    jointData_[dogIndex_+0].vel_ = convertKinDeg(current_vels.Actuator1);
    jointData_[dogIndex_+0].tau_ = joint_tqs.Actuator1;

    jointData_[dogIndex_+1].pos_ = kinova_angles.joint2 * M_PI/180;
    jointData_[dogIndex_+1].vel_ = convertKinDeg(current_vels.Actuator2);
    jointData_[dogIndex_+1].tau_ = joint_tqs.Actuator2;

    jointData_[dogIndex_+2].pos_ = kinova_angles.joint3 * M_PI/180;
    jointData_[dogIndex_+2].vel_ = convertKinDeg(current_vels.Actuator3);
    jointData_[dogIndex_+2].tau_ = joint_tqs.Actuator3;

    jointData_[dogIndex_+3].pos_ = kinova_angles.joint4 * M_PI/180;
    jointData_[dogIndex_+3].vel_ = convertKinDeg(current_vels.Actuator4);
    jointData_[dogIndex_+3].tau_ = joint_tqs.Actuator4;

    jointData_[dogIndex_+4].pos_ = kinova_angles.joint5 * M_PI/180;
    jointData_[dogIndex_+4].vel_ = convertKinDeg(current_vels.Actuator5);
    jointData_[dogIndex_+4].tau_ = joint_tqs.Actuator5;

    jointData_[dogIndex_+5].pos_ = kinova_angles.joint6 * M_PI/180;
    jointData_[dogIndex_+5].vel_ = convertKinDeg(current_vels.Actuator6);
    jointData_[dogIndex_+5].tau_ = joint_tqs.Actuator6;
}

void QmHW::KinovaCmdWrite() {
    // velocity
    AngularInfo joint_velocities;
    joint_velocities.Actuator1 = static_cast<float>(jointData_[dogIndex_+0].velDes_/M_PI*180.0);
    joint_velocities.Actuator2 = static_cast<float>(jointData_[dogIndex_+1].velDes_/M_PI*180.0);
    joint_velocities.Actuator3 = static_cast<float>(jointData_[dogIndex_+2].velDes_/M_PI*180.0);
    joint_velocities.Actuator4 = static_cast<float>(jointData_[dogIndex_+3].velDes_/M_PI*180.0);
    joint_velocities.Actuator5 = static_cast<float>(jointData_[dogIndex_+4].velDes_/M_PI*180.0);
    joint_velocities.Actuator6 = static_cast<float>(jointData_[dogIndex_+5].velDes_/M_PI*180.0);
    kinova_comm_->setJointVelocities(joint_velocities);

    // torque
//    for (size_t i = 0; i < 6; ++i) {
//        l_joint_torque_[i] = static_cast<float>(jointData_[dogIndex_+i].ff_);
//    }
//    kinova_comm_->setJointTorques(l_joint_torque_);
}

void QmHW::UnitreeCmdWrite() {
    for (int i = 0; i < 12; ++i) {
        lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
        lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
        lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
        lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
        lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
    }
    safety_->PositionLimit(lowCmd_);
    safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
    udp_->SetSend(lowCmd_);
    udp_->Send();
}

void QmHW::read(const ros::Time &time, const ros::Duration &period) {

    // Read unitree state
    UnitreeStateRead();

    // Read kinova state only 100hz
    // TODO: high resolution clock
    if(last_kinova_state_time_ + ros::Duration(1.0 / 100.0) < time)
    {
        readTimer_.startTimer();
        KinovaStateRead();
        readTimer_.endTimer();
        last_kinova_state_time_ = time;
    }

    // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto& name : names) {
        HybridJointHandle handle = hybridJointInterface_.getHandle(name);
        handle.setFeedforward(0.);
        handle.setVelocityDesired(0.);
        handle.setKd(3.);
    }

    // Debug
    publishState(time);
}

void QmHW::write(const ros::Time &time, const ros::Duration &period) {

    if(temperatureProtection())
    {
        stop();
        ROS_WARN("Motor overheats!");
    }

    UnitreeCmdWrite();

    // Only 100Hz cmd
    // TODO: high resolution clock
    if(last_kinova_cmd_time_ + ros::Duration(1.0 / 100.0) < time)
    {
        writeTimer_.startTimer();
        KinovaCmdWrite();
        writeTimer_.endTimer();
        last_kinova_cmd_time_ = time;
    }
}

bool QmHW::setupContactSensor(ros::NodeHandle &nh) {
    nh.getParam("contact_threshold", contactThreshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
    }
    return true;
}

bool QmHW::setupImu() {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                           imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                           imuData_.linearAccCov_));
    imuData_.oriCov_[0] = 0.0012;
    imuData_.oriCov_[4] = 0.0012;
    imuData_.oriCov_[8] = 0.0012;

    imuData_.angularVelCov_[0] = 0.0004;
    imuData_.angularVelCov_[4] = 0.0004;
    imuData_.angularVelCov_[8] = 0.0004;

    return true;
}

bool QmHW::setupJoints() {
    // legged joint
    for (const auto& joint : urdfModel_->joints_) {
        int leg_index = 0;
        int joint_index = 0;
        if (joint.first.find("RF") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::FR_;
        } else if (joint.first.find("LF") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::FL_;
        } else if (joint.first.find("RH") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::RR_;
        } else if (joint.first.find("LH") != std::string::npos) {
            leg_index = UNITREE_LEGGED_SDK::RL_;
        } else {
            continue;
        }

        if (joint.first.find("HAA") != std::string::npos) {
            joint_index = 0;
        } else if (joint.first.find("HFE") != std::string::npos) {
            joint_index = 1;
        } else if (joint.first.find("KFE") != std::string::npos) {
            joint_index = 2;
        } else {
            continue;
        }

        int index = leg_index * 3 + joint_index;
        hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                          &jointData_[index].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                               &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
    }

    arm_joint_names_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        arm_joint_names_[i] = "j2n6s300_joint_" + boost::lexical_cast<std::string>(i+1);
        hardware_interface::JointStateHandle state_handle(arm_joint_names_[i], &jointData_[dogIndex_+i].pos_,
                                                          &jointData_[dogIndex_+i].vel_, &jointData_[dogIndex_+i].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[dogIndex_+i].posDes_, &jointData_[dogIndex_+i].velDes_,
                                                               &jointData_[dogIndex_+i].kp_, &jointData_[dogIndex_+i].kd_, &jointData_[dogIndex_+i].ff_));
    }

    return true;
}

void QmHW::publishState(const ros::Time &time) {
    if (last_publish_time_ + ros::Duration(1.0 / 100.0) < time)
    {
        // IMU
        if(imuStatePub_->trylock()){
            imuStatePub_->msg_.header.stamp = ros::Time::now();
            imuStatePub_->msg_.header.frame_id = "imu_link";
            imuStatePub_->msg_.orientation.x = imuData_.ori_[0];
            imuStatePub_->msg_.orientation.y = imuData_.ori_[1];
            imuStatePub_->msg_.orientation.z = imuData_.ori_[2];
            imuStatePub_->msg_.orientation.w = imuData_.ori_[3];
            imuStatePub_->msg_.linear_acceleration.x = imuData_.linearAcc_[0];
            imuStatePub_->msg_.linear_acceleration.y = imuData_.linearAcc_[1];
            imuStatePub_->msg_.linear_acceleration.z = imuData_.linearAcc_[2];
            imuStatePub_->msg_.angular_velocity.x = imuData_.angularVel_[0];
            imuStatePub_->msg_.angular_velocity.y = imuData_.angularVel_[1];
            imuStatePub_->msg_.angular_velocity.z = imuData_.angularVel_[2];

            imuStatePub_->unlockAndPublish();
        }

        // Motor
        if(motorStatePub_->trylock()){
            qm_msgs::MotorState motor_state;
            motor_state.header.stamp = time;
            for (int i = 0; i < 18; ++i)
            {
                motor_state.q[i] = jointData_[i].pos_;
                motor_state.dq[i] = jointData_[i].vel_;
                motor_state.tau[i] = jointData_[i].tau_;
                motor_state.ff[i] = jointData_[i].ff_;
                motor_state.temperature[i] = jointData_[i].temperature_;
            }
            motorStatePub_->msg_ = motor_state;
            motorStatePub_->unlockAndPublish();
        }

        // ee force
        if(wrenchPub_->trylock())
        {
            wrenchPub_->msg_.header.stamp = time;
            wrenchPub_->msg_.wrench.force.x = eeForce_.x_;
            wrenchPub_->msg_.wrench.force.y = eeForce_.y_;
            wrenchPub_->msg_.wrench.force.z = eeForce_.z_;
            wrenchPub_->msg_.wrench.torque.x = 0.;
            wrenchPub_->msg_.wrench.torque.y = 0.;
            wrenchPub_->msg_.wrench.torque.z = 0.;
            wrenchPub_->unlockAndPublish();
        }
        last_publish_time_ = time;
    }
}

bool QmHW::loadUrdf(ros::NodeHandle &rootNh) {
    std::string urdfString;
    if (urdfModel_ == nullptr) {
        urdfModel_ = std::make_shared<urdf::Model>();
    }
    // get the urdf param on param server
    rootNh.getParam("qm_description", urdfString);
    return !urdfString.empty() && urdfModel_->initString(urdfString);
}


}