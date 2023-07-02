//
// Created by skywoodsz on 2023/5/12.
//

#include "qm_hw/QmHW.h"
#include "qm_hw/QmHWLoop.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "qm_hw");
    ros::NodeHandle nh;
    ros::NodeHandle robotHwNh("~");

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("\033[1;32m Main Loop Start! \033[0m");
    try{
        std::shared_ptr<qm::QmHW> qmHw = std::make_shared<qm::QmHW>();

        qmHw->init(nh, robotHwNh);

        qm::QmHWLoop controlLoop(nh, qmHw);

        ros::waitForShutdown();
    } catch (const ros::Exception& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
                                 << "\t" << e.what());
        return 1;
    }

    return 0;
}