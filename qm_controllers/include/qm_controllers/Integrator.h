//
// Created by skywoodsz on 2023/6/11.
//

#ifndef SRC_INTEGRATOR_H
#define SRC_INTEGRATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Core>

namespace qm{
class Integrator
{
public:
    Integrator(){};
    Integrator(int rows);

    void SetSize(int rows);
    void Reset();

    void Integrate(Eigen::VectorXd vec, const double dt);
    Eigen::VectorXd GetIntegral();
    void SetIntegral(Eigen::VectorXd target);
    void SetLimit(Eigen::VectorXd limit);

private:
    Eigen::VectorXd integral_;
    Eigen::VectorXd limit_;
};
}

#endif //SRC_INTEGRATOR_H
