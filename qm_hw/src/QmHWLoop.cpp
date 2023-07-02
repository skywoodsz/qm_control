//
// Created by skywoodsz on 2023/5/12.
//

#include "qm_hw/QmHWLoop.h"

namespace qm{

QmHWLoop::QmHWLoop(ros::NodeHandle &nh, std::shared_ptr<QmHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true)
{
    // Create the controller manager
    controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

    // Load ros params
    int error = 0;
    int threadPriority = 0;
    ros::NodeHandle nhP("~");
    error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
    error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
    error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
    if (error > 0) {
        std::string error_message =
                "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
        ROS_ERROR_STREAM(error_message);
        throw std::runtime_error(error_message);
    }

    // Get current time for use with first update
    lastTime_ = Clock::now();

    // Setup loop thread
    loopThread_ = std::thread([&]() {
        while (loopRunning_) {
            update();
        }
    });

    // System
    sched_param sched{.sched_priority = threadPriority};
    if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
        ROS_WARN(
                "Failed to set threads priority (one possible reason could be that the user and the group permissions "
                "are not set properly.).\n");
    }
}

void QmHWLoop::update() {
    const auto currentTime = Clock::now();
    const Duration desiredDuration(1.0 / loopHz_);

    // Get change in time
    Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
    elapsedTime_ = ros::Duration(time_span.count());
    lastTime_ = currentTime;

    // Check cycle time for excess delay
    const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
    if (cycle_time_error > cycleTimeErrorThreshold_) {
        ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                                   << "cycle time: " << elapsedTime_ << "s, "
                                                                   << "threshold: " << cycleTimeErrorThreshold_ << "s");
    }

    // Input
    // get the hardware's state
    hardwareInterface_->read(ros::Time::now(), elapsedTime_);

    // Control
    // let the controller compute the new command (via the controller manager)
    controllerManager_->update(ros::Time::now(), elapsedTime_);

    // Output
    // send the new command to hardware
    hardwareInterface_->write(ros::Time::now(), elapsedTime_);

    // Sleep
    const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
    std::this_thread::sleep_until(sleepTill);
}

QmHWLoop::~QmHWLoop() {
    loopRunning_ = false;
    hardwareInterface_->stop();
    if(loopThread_.joinable()){
        loopThread_.join();
    }
}

}
