//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_MODELSETTINGS_H
#define SRC_MODELSETTINGS_H

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace qm{
using namespace ocs2;

struct QMModelInfo{
    std::string baseFrame;                      // name of the root frame of the robot
    std::string eeFrame;                        // name of the end-effector frame of the robot
};

struct ModelSettings {
    scalar_t positionErrorGain = 0.0;

    scalar_t phaseTransitionStanceTime = 0.4;

    bool verboseCppAd = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/ocs2";

    // This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
    std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                        "j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3",
                                        "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"};

    std::vector<std::string> contactNames6DoF{};
    std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // For mobile manipulator
    QMModelInfo info;
};

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldName = "model_settings", bool verbose = "true");

}

#endif //SRC_MODELSETTINGS_H
