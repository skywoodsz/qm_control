//
// Created by skywoodsz on 2023/5/12.
//

#ifndef SRC_QMHWLOOP_H
#define SRC_QMHWLOOP_H

#include <chrono>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "qm_hw/QmHW.h"

namespace qm{

class QmHWLoop{
using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;

public:
    QmHWLoop(ros::NodeHandle& nh, std::shared_ptr<QmHW> hardware_interface);
    ~QmHWLoop();

    void update();

private:
    // ROS Param
    ros::NodeHandle nh_;

    // Timer Param
    double cycleTimeErrorThreshold_{}, loopHz_{};
    std::thread loopThread_;
    std::atomic_bool loopRunning_{};
    ros::Duration elapsedTime_;
    Clock::time_point lastTime_;

    // Controller Manager
    std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

    std::shared_ptr<QmHW> hardwareInterface_;
};

}

#endif //SRC_QMHWLOOP_H
