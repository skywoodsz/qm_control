//
// Created by skywoodsz on 2023/5/3.
//

#ifndef SRC_BOUNDEDADMITTANCEWITHK_H
#define SRC_BOUNDEDADMITTANCEWITHK_H

#include "qm_compliant/CompliantBase.h"
#include <std_msgs/Float64.h>

namespace qm{
using namespace ocs2;

class BoundedAdmittanceWithK {
public:
    BoundedAdmittanceWithK(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                      const PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t update(const vector_t &rbdStateMeasured, scalar_t time, scalar_t period);
    void initParam();
    void setJointIdx(size_t idx);
    void setTorqueMax(scalar_t tau_max);
    void setParam(scalar_t M, scalar_t K, scalar_t B, scalar_t L, scalar_t Mx, scalar_t Bx, scalar_t Kx);
    void setDesired(scalar_t ddot_q0, scalar_t dot_q0, scalar_t q0);
    void setForceDesired(scalar_t force) { fd_ = force; };
    void setForceFeedback(scalar_t force) { f_ = force; };
    vector_t getProxyState();
private:
    scalar_t projectionFunction(scalar_t x);

    size_t idx_;
    scalar_t T_;
    // param
    scalar_t Mx_, Bx_, Kx_, f_, fd_; // proxy
    scalar_t ddot_q0_, dot_q0_, q0_;
    scalar_t M_, B_, K_, L_; // controller
    scalar_t tau_max_;

    scalar_t ux_star_, qx_start_, phi_b_, phi_a_, qs_star_, tau_star_, tau_, qx_, ux_, a_, qs_, ax_;
    scalar_t ux_pre_, qx_pre_, qs_pre_, a_pre_, tau_pre_;

    // flag
    bool init_flag_, idx_flag_, param_flag_, tau_flag_;

    size_t generalizedCoordinatesNum_{};

    ros::Publisher qx_pub_, f_pub_, tau_pub_;
};

}




#endif //SRC_BOUNDEDADMITTANCEWITHK_H
