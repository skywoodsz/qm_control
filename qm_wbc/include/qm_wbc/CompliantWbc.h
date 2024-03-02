//
// Created by skywoodsz on 2023/5/6.
//

#ifndef SRC_COMPLIANTWBC_H
#define SRC_COMPLIANTWBC_H

#include "qm_wbc/WbcBase.h"

#include <qm_compliant/BoundedAdmittance.h>
#include <qm_compliant/BoundedAdmittanceWithK.h>
#include <qm_compliant/CartesianImpendance.h>

#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include <dynamic_reconfigure/server.h>
#include <qm_wbc/CompliantConfig.h>

namespace qm{
class CompliantWbc : public WbcBase{
    using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
public:
    CompliantWbc(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                 const PinocchioEndEffectorKinematics &eeKinematics,
                 const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                 ros::NodeHandle &controller_nh);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode, scalar_t period, scalar_t time) override;
    vector_t getImpDesiredTorque(){ return imp_desired_; }
private:
    void dynamicCallback(qm_wbc::CompliantConfig& config, uint32_t /*level*/);

    void BoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                               const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    void BaseBoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector6_t BoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period, vector_t imp);
    vector6_t BaseBoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,
                                          vector_t imp, scalar_t force_z);
    vector_t BoundanceAdmittanceControl(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                               size_t mode, scalar_t period, scalar_t time);

    // impendance controller
    std::shared_ptr<CartesianImpendance> impendace_controller_;
    // bounded admittance
    std::shared_ptr<BoundedAdmittance> bounded_admittance_controller2_;
    std::shared_ptr<BoundedAdmittance> bounded_admittance_controller1_;
    std::shared_ptr<BoundedAdmittanceWithK> bounded_admittance_controller_base_x_;

    // param
    std::shared_ptr<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>> dynamic_srv_{};

    vector6_t tau_max_;
    vector_t imp_desired_;
    scalar_t Mbase_px_, Bbase_px_, Kbase_px_; // base proxy
    scalar_t Mbase_py_, Bbase_py_, Kbase_py_; // base proxy
    scalar_t Bbasex_, Lbasex_, Kbasex_, Mbasex_; // base controller
    scalar_t Bbasey_, Lbasey_, Kbasey_, Mbasey_; // base controller
    scalar_t Mbx_, Bbx_; // proxy
    scalar_t Bb2_, Lb2_, Kb2_, Mb2_; // controller
    scalar_t Bb1_, Lb1_, Kb1_, Mb1_; // controller
    scalar_t mu_{};
};
}

#endif //SRC_COMPLIANTWBC_H
