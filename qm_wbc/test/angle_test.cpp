//
// Created by skywoodsz on 2023/3/26.
//
#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;

int main(int argc, char* argv[])
{


    ::ros::init(argc, argv, "angle_test");
    ::ros::NodeHandle nodeHandle;

    const Eigen::Matrix<scalar_t, 3, 3> rotationMatrixLhs = Eigen::Matrix3d::Identity();
    const Eigen::Matrix<scalar_t, 3, 3>& rotationMatrixRhs = Eigen::Matrix3d::Identity();
    vector_t error = rotationErrorInWorld<scalar_t>(rotationMatrixLhs, rotationMatrixRhs);

    std::cout<<error<<std::endl;

    ros::spin();
    return 0;
}
