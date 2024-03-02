//
// Created by skywoodsz on 2023/4/22.
//

#ifndef SRC_CARTESIANIMPENDANCE_H
#define SRC_CARTESIANIMPENDANCE_H

#include "qm_compliant/CompliantBase.h"

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>

#include <dynamic_reconfigure/server.h>
#include <qm_compliant/ImpemdanceConfig.h>

namespace qm{
using namespace ocs2;

class CartesianImpendance : public CompliantBase{
public:
    CartesianImpendance(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t update(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    void setDesiredPosition(vector3_t xd);
    void setDesiredVelocity(vector3_t vd){ vd_ = vd; }
private:
    vector_t calculateCommandedTorques(scalar_t period);
    void dynamicCallback(qm_compliant::ImpemdanceConfig& config, uint32_t /*level*/);

    PinocchioInterface pinocchioInterfaceImpendance_;

    std::shared_ptr<dynamic_reconfigure::Server<qm_compliant::ImpemdanceConfig>> dynamic_srv_{};

    matrix_t D_, K_;
    matrix_t Dr_, Kr_;
    vector3_t xd_, rd_, vd_;

    // flag
    bool desired_init_flag_;
};
}



#endif //SRC_CARTESIANIMPENDANCE_H
