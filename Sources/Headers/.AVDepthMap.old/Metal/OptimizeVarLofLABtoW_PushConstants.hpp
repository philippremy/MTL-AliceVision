//
//  OptimizeVarLofLABtoW_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef OptimizeVarLofLABtoW_PushConstants_h
#define OptimizeVarLofLABtoW_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct OptimizeVarLofLABtoW_PushConstants
{
    unsigned int rcLevelWidth;
    unsigned int rcLevelHeight;
    float rcMipmapLevel;
    int stepXY;
    ROI roi;
};

}
}

#endif /* OptimizeVarLofLABtoW_PushConstants_h */
