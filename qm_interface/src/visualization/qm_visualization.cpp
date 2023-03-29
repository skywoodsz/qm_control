//
// Created by skywoodsz on 2023/3/5.
//

#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// ocs2
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "qm_interface/visualization/qm_visualization.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

QmVisualizer::QmVisualizer(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo centroidalModelInfo,
                           ModelSettings modelInfo, const ocs2::PinocchioEndEffectorKinematics &endEffectorKinematics,
                           const ocs2::PinocchioEndEffectorKinematics &armEndEffectorKinematics,
                           ros::NodeHandle &nodeHandle, ocs2::scalar_t maxUpdateFrequency)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          centroidalModelInfo_(std::move(centroidalModelInfo)),
          modelInfo_(modelInfo),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          armEffectorKinematicsPtr_(armEndEffectorKinematics.clone()),
          lastTime_(std::numeric_limits<scalar_t>::lowest()),
          minPublishTimeDifference_(1.0 / maxUpdateFrequency){
    endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
    armEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
    launchVisualizerNode(nodeHandle);
}

void QmVisualizer::launchVisualizerNode(ros::NodeHandle &nodeHandle) {
    costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/qm/desiredBaseTrajectory", 1);
    costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
    costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/qm/desiredFeetTrajectory/LF", 1);
    costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/qm/desiredFeetTrajectory/RF", 1);
    costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/qm/desiredFeetTrajectory/LH", 1);
    costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/qm/desiredFeetTrajectory/RH", 1);
    stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/qm/optimizedStateTrajectory", 1);
    currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/qm/currentState", 1);

    // Load URDF model
    urdf::Model urdfModel;
    if (!urdfModel.initParam("qm_description")) {
        std::cerr << "[QmVisualizer] Could not read URDF from: \"qm_description\"" << std::endl;
    } else {
        KDL::Tree kdlTree;
        kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

        robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
        robotStatePublisherPtr_->publishFixedTransforms(true);
    }
}

void QmVisualizer::update(const ocs2::SystemObservation &observation, const ocs2::PrimalSolution &primalSolution,
                              const ocs2::CommandData &command) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto timeStamp = ros::Time::now();
        publishObservation(timeStamp, observation);
        publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
        publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                        primalSolution.modeSchedule_);
        lastTime_ = observation.time;
    }

}

void QmVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const ocs2::scalar_array_t &mpcTimeTrajectory,
                                                  const ocs2::vector_array_t &mpcStateTrajectory,
                                                  const ocs2::ModeSchedule &modeSchedule) {
    if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
        return;  // Nothing to publish
    }

    visualization_msgs::MarkerArray markerArray;

    // Base trajectory
    std::vector<geometry_msgs::Point> baseTrajectory;
    baseTrajectory.reserve(mpcStateTrajectory.size());

    // Reserve Feet msg
    feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
    std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

    // End effector trajectory
    std::vector<geometry_msgs::Point> endEffectorTrajectory;
    endEffectorTrajectory.reserve(mpcStateTrajectory.size());

    // Extract Com, Feet and EE from state
    std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

        // Fill com position and pose msgs
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());
        baseTrajectory.push_back(pose.position);

        // Fill feet msgs
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            const auto position = getPointMsg(feetPositions[i]);
            feetMsgs[i].push_back(position);
        }

        // Fill EE msgs
        const auto EePositions =  armEffectorKinematicsPtr_->getPosition(state);
        const auto EePosition = getPointMsg(EePositions[0]);
        endEffectorTrajectory.push_back(EePosition);
    });

    markerArray.markers.emplace_back(getLineMsg(std::move(baseTrajectory), Color::red, trajectoryLineWidth_));
    markerArray.markers.back().ns = "CoM Trajectory";

    markerArray.markers.emplace_back(getLineMsg(std::move(endEffectorTrajectory), Color::blue, trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectory";

    // Convert feet msgs to Array message
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
        markerArray.markers.back().ns = "Feet Trajectories";
    }

    // Future footholds
    visualization_msgs::Marker sphereList;
    sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereList.scale.x = footMarkerDiameter_;
    sphereList.scale.y = footMarkerDiameter_;
    sphereList.scale.z = footMarkerDiameter_;
    sphereList.ns = "Future footholds";
    sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
    const auto& eventTimes = modeSchedule.eventTimes;
    const auto& subsystemSequence = modeSchedule.modeSequence;
    const auto tStart = mpcTimeTrajectory.front();
    const auto tEnd = mpcTimeTrajectory.back();
    for (size_t event = 0; event < eventTimes.size(); ++event) {
        if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
            const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
            const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
            const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

            const auto& model = pinocchioInterface_.getModel();
            auto& data = pinocchioInterface_.getData();
            pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
            pinocchio::updateFramePlacements(model, data);

            const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
            for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
                if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
                    sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
                    sphereList.colors.push_back(getColor(feetColorMap_[i]));
                }
            }
        }
    }
    markerArray.markers.push_back(std::move(sphereList));


    assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

    stateOptimizedPublisher_.publish(markerArray);
}

/**
 * vis desired base/foot traj.
 */
void QmVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const ocs2::TargetTrajectories &targetTrajectories) {
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;
    const auto& inputTrajectory = targetTrajectories.inputTrajectory;

    // Reserve com messages
    std::vector<geometry_msgs::Point> desiredBasePositionMsg;
    desiredBasePositionMsg.reserve(stateTrajectory.size());

    // Reserve feet messages
    feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
    }

    for (size_t j = 0; j < stateTrajectory.size(); j++) {
        const auto state = stateTrajectory.at(j);
        vector_t input(centroidalModelInfo_.inputDim);
        if (j < inputTrajectory.size()) {
            input = inputTrajectory.at(j);
        } else {
            input.setZero();
        }

        // Construct base pose msg
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());

        // Fill message containers
        desiredBasePositionMsg.push_back(pose.position);

        // Fill feet msgs
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            geometry_msgs::Pose footPose;
            footPose.position = getPointMsg(feetPositions[i]);
            desiredFeetPositionMsgs[i].push_back(footPose.position);
        }
    }

    // Headers
    auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
    comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    comLineMsg.id = 0;

    // Publish
    costDesiredBasePositionPublisher_.publish(comLineMsg);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
        footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
        footLineMsg.id = 0;
        costDesiredFeetPositionPublishers_[i].publish(footLineMsg);
    }

    // ee command tf
    const auto EeStateTrajectory = stateTrajectory.back().tail(7);
    const Eigen::Vector3d eeDesiredPosition = EeStateTrajectory.head(3);
    Eigen::Quaterniond eeDesiredOrientation;
    eeDesiredOrientation.coeffs() = EeStateTrajectory.tail(4);
    geometry_msgs::TransformStamped command_tf;
    command_tf.header.stamp = timeStamp;
    command_tf.header.frame_id = frameId_;
    command_tf.child_frame_id = "command";
    command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
    command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
    tfBroadcaster_.sendTransform(command_tf);
}

void QmVisualizer::publishObservation(ros::Time timeStamp, const ocs2::SystemObservation &observation) {
    // Extract components from state
    const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
    const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

    // Compute cartesian state and inputs
    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
    std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
    }

    // Publish
    publishJointTransforms(timeStamp, qJoints);
    publishBaseTransform(timeStamp, basePose);
    publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
}

/**
 * vis foot force; cop; support zone
 */
void QmVisualizer::publishCartesianMarkers(ros::Time timeStamp, const ocs2::legged_robot::contact_flag_t &contactFlags,
                                          const std::vector<vector3_t> &feetPositions,
                                          const std::vector<vector3_t> &feetForces) const {
    // Reserve message
    const size_t numberOfCartesianMarkers = 10;
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.reserve(numberOfCartesianMarkers);

    // Feet positions and Forces
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
        markerArray.markers.emplace_back(
                getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
        markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
    }

    // Center of pressure
    markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                               contactFlags.begin(), Color::green, copMarkerDiameter_));

    // Support polygon
    markerArray.markers.emplace_back(
            getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

    // Give markers an id and a frame
    assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

    // Publish cartesian markers (minus the CoM Pose)
    currentStatePublisher_.publish(markerArray);
}

void QmVisualizer::publishBaseTransform(ros::Time timeStamp, const ocs2::vector_t &basePose) {
    if (robotStatePublisherPtr_ != nullptr) {
        geometry_msgs::TransformStamped baseToWorldTransform;
        baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
        baseToWorldTransform.child_frame_id = "base";

        const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
        baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
        baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
        tfBroadcaster_.sendTransform(baseToWorldTransform);
    }
}

void QmVisualizer::publishJointTransforms(ros::Time timeStamp, const ocs2::vector_t &jointAngles) const {
    if (robotStatePublisherPtr_ != nullptr) {
        std::map<std::string, scalar_t> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                                       {"LH_HAA", jointAngles[3]}, {"LH_HFE", jointAngles[4]},  {"LH_KFE", jointAngles[5]},
                                                       {"RF_HAA", jointAngles[6]}, {"RF_HFE", jointAngles[7]},  {"RF_KFE", jointAngles[8]},
                                                       {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]},
                                                       {"j2n6s300_joint_1", jointAngles[12]}, {"j2n6s300_joint_2", jointAngles[13]},
                                                       {"j2n6s300_joint_3", jointAngles[14]}, {"j2n6s300_joint_4", jointAngles[15]},
                                                       {"j2n6s300_joint_5", jointAngles[16]}, {"j2n6s300_joint_6", jointAngles[17]}
        };
        robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
    }
}

}