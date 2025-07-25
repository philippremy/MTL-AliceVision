//
//  RefineBestDepth_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef RefineBestDepth_PushConstants_h
#define RefineBestDepth_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct RefineBestDepth_PushConstants
{
    int volDimZ;
    int samplesPerPixSize;
    int halfNbSamples;
    int halfNbDepths;
    float twoTimesSigmaPowerTwo;
    ROI roi;
};

}
}

#endif /* RefineBestDepth_PushConstants_h */
