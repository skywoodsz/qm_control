//
// Created by skywoodsz on 2023/2/19.
//
#ifndef SRC_MATHCONVERT_H
#define SRC_MATHCONVERT_H

#include <iostream>
#include <numeric>
#include <functional>
#include <queue>
#include <ocs2_core/Types.h>


using namespace ocs2;

class MovingAverage {
public:
    MovingAverage(int winSize, int dataSize){
        pre_.resize(dataSize);
        winSize_ = winSize;
    };
    vector_t filter(vector_t val, int winSize)
    {
        if(val.size() != pre_.size())
        {
            std::cerr<<"[MovingAverage]: data size is wrong!"<<std::endl;
            return vector_t::Zero(pre_.size());
        }

        winSize_ = winSize;

        dataQueue_.push(val);
        if(dataQueue_.size() > winSize_)
        {
            vector_t tmp = dataQueue_.front();
            pre_ -= tmp;
            dataQueue_.pop();
        }
        pre_ += val;
        return pre_ / dataQueue_.size();
    };
private:
    int winSize_;
    std::queue<vector_t> dataQueue_;
    vector_t pre_;
};



template <typename T>
T square(T a) {
    return a * a;
}

/**
 * convert quat to eular angle
 * @param quat: w, x, y, z
 * @return yaw, roll, pitch
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

//vector_t LimitJointAngle(vector_t joint_angle)
//{
//    vector_t joint_angle_limit(joint_angle.size());
//    for (int i = 0; i < joint_angle.size(); ++i) {
//         if(joint_angle[i] > M_PI || joint_angle[i] < -M_PI)
//         {
//             joint_angle_limit[i] = fmod(joint_angle[i], M_PI);
//         }
//         else
//         {
//             joint_angle_limit[i] = joint_angle[i];
//         }
//    }
//    return joint_angle_limit;
//}

#endif //SRC_MATHCONVERT_H