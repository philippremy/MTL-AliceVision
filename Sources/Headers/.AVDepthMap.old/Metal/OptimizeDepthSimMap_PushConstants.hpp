//
//  OptimizeDepthSimMap_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef OptimizeDepthSimMap_PushConstants_h
#define OptimizeDepthSimMap_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct OptimizeDepthSimMap_PushConstants
{
    int rcDeviceCameraParamsId;
    int iter;
    ROI roi;
};

}
}

#endif /* OptimizeDepthSimMap_PushConstants_h */
