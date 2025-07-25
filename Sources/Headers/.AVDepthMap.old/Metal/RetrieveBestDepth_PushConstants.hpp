//
//  RetrieveBestDepth_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef RetrieveBestDepth_PushConstants_h
#define RetrieveBestDepth_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct RetrieveBestDepth_PushConstants
{
    bool hasDepthSimTex;
    int rcDeviceCameraParamsId;
    int volDimZ;
    int scaleStep;
    float thicknessMultFactor;
    float maxSimilarity;
    Range depthRange;
    ROI roi;
};

}
}

#endif /* RetrieveBestDepth_PushConstants_h */
