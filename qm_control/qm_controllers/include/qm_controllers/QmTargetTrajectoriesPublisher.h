//
// Created by skywoodsz on 2023/3/3.
//

#ifndef SRC_QMTARGETTRAJECTORIESPUBLISHER_H
#define SRC_QMTARGETTRAJECTORIESPUBLISHER_H

#include <functional>
#include <memory>
#include <mutex>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <qm_msgs/ee_state.h>
#include <geometry_msgs/Twist.h>

namespace qm{
using namespace ocs2;

class QmTargetTrajectoriesInteractiveMarker final{
public:
    using GoalPoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
            const SystemObservation& observation, const SystemObservation& eeState)>;

    using CmdToTargetTrajectories = std::function<TargetTrajectories(
            const vector_t& cmd, vector_t& lastEeTarget,
            const SystemObservation& observation, const SystemObservation& eeState)>;

    QmTargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nh, const std::string& topicPrefix,
                                          GoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
                                          CmdToTargetTrajectories cmdVelToTargetTrajectories,
                                          CmdToTargetTrajectories eeCmdVelToTargetTrajectories)
    : server_("simple_marker"),
      goalPoseToTargetTrajectories_(std::move(goalPoseToTargetTrajectories)),
      cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
      eeCmdVelToTargetTrajectories_(std::move(eeCmdVelToTargetTrajectories))
    {
        // Trajectories publisher
        targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

        // observation subscriber
        auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(latestObservationMutex_);
            latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        };
        observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

        lastEeTarget_ = vector_t::Zero(7);
        lastEeTarget_.head(3) << 0.52, 0.09, 0.44;
        lastEeTarget_.tail(4) << Eigen::Quaternion<scalar_t>(-0.5, 0.5, -0.5, 0.5).coeffs();

        // current ee pose
        auto eePoseCallback = [this](const qm_msgs::ee_state::ConstPtr& msg){
            std::lock_guard<std::mutex> lock(latestObservationEeMutex_);
            {
                qm_msgs::ee_state eeState = *msg;
                latestObservationEe_.time = eeState.time;
                latestObservationEe_.state.resize(eeState.state.size());
                for (size_t i = 0; i < eeState.state.size(); i++) {
                    const auto& state = eeState.state[i];
                    latestObservationEe_.state[i] = static_cast<scalar_t>(state);
                }
            }
        };
        eePoseSub_ = nh.subscribe<qm_msgs::ee_state>(topicPrefix + "_mpc_observation_ee_state", 1, eePoseCallback);

        // ee cmd vel subscriber
        auto eeCmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
            if (latestObservation_.time == 0.0) {
                return;
            }

            vector_t cmdVel = vector_t::Zero(3);
            cmdVel[0] = msg->linear.x;
            cmdVel[1] = msg->linear.y;
            cmdVel[2] = msg->linear.z;

            const auto trajectories = eeCmdVelToTargetTrajectories_(cmdVel, lastEeTarget_,
                                                      latestObservation_, latestObservationEe_);
            targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        };
        eeCmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/ee_cmd_vel", 1, eeCmdVelCallback);

        // cmd_vel subscriber
        auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
            if (latestObservation_.time == 0.0) {
                return;
            }

            vector_t cmdVel = vector_t::Zero(4);
            cmdVel[0] = msg->linear.x;
            cmdVel[1] = msg->linear.y;
            cmdVel[2] = msg->linear.z;
            cmdVel[3] = msg->angular.z;

            const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, lastEeTarget_,
                                                  latestObservation_, latestObservationEe_);
            targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        };
        dogCmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

        // interactive marker
        menuHandler_.insert("Send target pose", boost::bind(&QmTargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
        auto interactiveMarker = createInteractiveMarker();
        server_.insert(interactiveMarker);
        menuHandler_.apply(server_, interactiveMarker.name);
        server_.applyChanges();
    }

private:
    visualization_msgs::InteractiveMarker createInteractiveMarker() const;
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

    ::ros::Subscriber observationSub_, eePoseSub_, dogCmdVelSub_, eeCmdVelSub_;

    mutable std::mutex latestObservationMutex_, latestObservationEeMutex_;
    SystemObservation latestObservation_, latestObservationEe_;

    interactive_markers::MenuHandler menuHandler_;
    interactive_markers::InteractiveMarkerServer server_;
    GoalPoseToTargetTrajectories goalPoseToTargetTrajectories_;
    CmdToTargetTrajectories cmdVelToTargetTrajectories_;
    CmdToTargetTrajectories eeCmdVelToTargetTrajectories_;
    vector_t lastEeTarget_;
};
}




#endif //SRC_QMTARGETTRAJECTORIESPUBLISHER_H
