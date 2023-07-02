//
// Created by skywoodsz on 2023/6/11.
//

#include "qm_controllers/Integrator.h"

namespace qm{

Integrator::Integrator(int rows) {
    SetSize(rows);
    Reset();
}

void Integrator::SetSize(int rows) {
    integral_.resize(rows);
    limit_.resize(rows);
}

void Integrator::Reset() {
    integral_.setZero();
    limit_.setZero();
}

Eigen::VectorXd Integrator::GetIntegral() {
    return integral_;
}

void Integrator::Integrate(Eigen::VectorXd vec, const double dt) {
    integral_ += dt * vec;

    for (size_t i = 0; i < integral_.size(); ++i) {
        if(integral_(i) >= limit_(i))
            integral_(i) = limit_(i);
        if(integral_(i) <= -limit_(i))
            integral_(i) = -limit_(i);
    }
}

void Integrator::SetIntegral(Eigen::VectorXd target) {
    integral_ = target;
}

void Integrator::SetLimit(Eigen::VectorXd limit) {
    limit_ = limit;
}

}
