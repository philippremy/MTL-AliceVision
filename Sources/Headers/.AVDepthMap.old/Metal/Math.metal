//
//  Math.metal
//  AVDepthMapMetalKernels
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef MATH_h
#define MATH_h

#include <metal_stdlib>
using namespace metal;

namespace aliceVision {
namespace depthMap {

template<typename T>
inline T cbrt(T f) {
    return pow(f, 1.0 / 3.0);
}

}
}

#endif
