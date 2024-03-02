//
// Created by skywoodsz on 2023/4/25.
//

#ifndef SRC_ADMITTANCE_H
#define SRC_ADMITTANCE_H

#include "qm_compliant/CompliantBase.h"
#include <std_msgs/Float64.h>

namespace qm {
using namespace ocs2;

class Admittance : public CompliantBase{
public:
    Admittance(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
               const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t update(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);

    void initParam();
    size_t getJointIdx(){ return idx_; };
    void setJointIdx(size_t idx);
    void setTorqueMax(size_t tau_max){ tau_max_ = tau_max; };

    void setParam(scalar_t M, scalar_t K, scalar_t B, scalar_t L, scalar_t Mx, scalar_t Bx, scalar_t Kx);

    void setForceDesired(scalar_t force){ fd_ = force; };
    void setForceFeedback(scalar_t force){ f_ = force; };
    void setQ0(scalar_t q0){ q0_ = q0; };
    vector_t getProxyState();
    
protected:
    // math
    scalar_t projectionFunction(scalar_t x);

    // joind id
    size_t idx_;

    // param
    scalar_t Mx_, Bx_, Kx_, f_, fd_; // proxy
    scalar_t B_, L_, K_, M_, Khat_; // controller
    scalar_t tau_max_;

    //
    scalar_t ddot_qx_, dot_qx_, qx_, a_, tau_;
    scalar_t dot_q0_, q0_;
    scalar_t T_;

    // flag
    bool init_flag_, idx_flag_, param_flag_;

    // debug
    ros::Publisher qx_pub_, f_pub_, tau_pub_;
};

}

#endif //SRC_ADMITTANCE_H
