//
// Created by skywoodsz on 2023/5/1.
//

#ifndef SRC_BASEADMITTANCE_H
#define SRC_BASEADMITTANCE_H

#include "qm_compliant/Admittance.h"
#include <std_msgs/Float64.h>

namespace qm{
using namespace ocs2;

class BaseAdmittance : public Admittance{
public:
    BaseAdmittance(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t update(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period) override;
private:
    ros::Publisher tau_pub_;
};

}




#endif //SRC_BASEADMITTANCE_H
