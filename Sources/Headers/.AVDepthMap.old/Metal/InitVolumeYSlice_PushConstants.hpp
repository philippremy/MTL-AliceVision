//
//  InitVolumeYSlice_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef InitVolumeYSlice_PushConstants_h
#define InitVolumeYSlice_PushConstants_h

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#else
#include <simd/vector.h>
using namespace simd;
#endif

namespace aliceVision {
namespace depthMap {

struct InitVolumeYSlice_PushConstants
{
    int3 axisT;
    int3 volDim;
    int y;
    float cst;
};

}
}

#endif /* InitVolumeYSlice_PushConstants_h */
