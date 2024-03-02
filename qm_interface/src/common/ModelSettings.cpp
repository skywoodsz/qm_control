//
// Created by skywoodsz on 2023/2/21.
//

#include "qm_interface/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace qm{
using namespace ocs2;

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldName, bool verbose){
    ModelSettings modelSettings;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    if (verbose) {
        std::cerr << "\n #### Robot Model Settings:";
        std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, modelSettings.positionErrorGain, fieldName + ".positionErrorGain", verbose);
    loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime, fieldName + ".phaseTransitionStanceTime", verbose);

    loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);
    loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd", verbose);
    loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);

    loadData::loadPtreeValue<std::string>(pt, modelSettings.info.baseFrame, fieldName + ".baseFrame", verbose);
    loadData::loadPtreeValue<std::string>(pt, modelSettings.info.eeFrame, fieldName + ".eeFrame", verbose);

    if (verbose) {
        std::cerr << " #### =============================================================================" << std::endl;
    }

    return modelSettings;
}

}