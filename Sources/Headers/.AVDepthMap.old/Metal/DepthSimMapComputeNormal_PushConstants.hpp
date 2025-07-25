//
//  DepthMapSimMapComputeNormal_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef DepthSimMapComputeNormal_PushConstants_h
#define DepthSimMapComputeNormal_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct DepthSimMapComputeNormal_PushConstants
{
    int rcDeviceCameraParamsId;
    int stepXY;
    int TWsh;
    ROI roi;
};

}
}

#endif /* DepthSimMapComputeNormal_PushConstants_h */
