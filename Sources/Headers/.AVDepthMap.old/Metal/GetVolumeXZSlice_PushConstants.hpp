//
//  GetVolumeXZSlice_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef GetVolumeXZSlice_PushConstants_h
#define GetVolumeXZSlice_PushConstants_h

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#else
#include <simd/vector.h>
using namespace simd;
#endif

namespace aliceVision {
namespace depthMap {

struct GetVolumeXZSlice_PushConstants
{
    int3 axisT;
    int3 volDim;
    int y;
};

}
}

#endif /* GetVolumeXZSlice_PushConstants_h */
