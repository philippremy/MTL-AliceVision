//
//  AggregateCostVolumeAtXInSlices_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef AggregateCostVolumeAtXInSlices_PushConstants_h
#define AggregateCostVolumeAtXInSlices_PushConstants_h

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#else
#include <simd/vector.h>
using namespace simd;
#endif

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct AggregateCostVolumeAtXInSlices_PushConstants
{
    unsigned int rcSgmLevelWidth;
    unsigned int rcSgmLevelHeight;
    float rcMipmapLevel;
    int3 volDim;
    int3 axisT;
    float step;
    int y;
    float P1;
    float _P2;
    int ySign;
    int filteringIndex;
    ROI roi;
};

}
}

#endif /* AggregateCostVolumeAtXInSlices_PushConstants_h */
