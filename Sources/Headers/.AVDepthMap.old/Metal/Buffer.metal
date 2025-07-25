//
//  Buffer.metal
//  AVDepthMapMetalKernels
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef BUFFER_h
#define BUFFER_h

#include <metal_stdlib>
using namespace metal;

namespace aliceVision {
namespace depthMap {

inline float multi_fminf(float a, float b, float c)
{
  return min(min(a, b), c);
}

inline float multi_fminf(float a, float b, float c, float d)
{
  return min(min(min(a, b), c), d);
}

inline float getGauss(constant int* d_gaussianArrayOffset, constant float* d_gaussianArray, int scale, int idx) {
    return d_gaussianArray[d_gaussianArrayOffset[scale] + idx];
}

}
}

#endif
