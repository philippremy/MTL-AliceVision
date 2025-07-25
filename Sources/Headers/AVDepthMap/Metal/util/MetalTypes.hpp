// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#if defined(__METAL__)
    #include <metal_stdlib>
    using namespace metal;
#else
    #include <simd/vector.h>
    using namespace simd;
#endif

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
#define TSIM_REFINE_USE_HALF

namespace aliceVision {
namespace depthMap {

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
using MTLColorBaseType = unsigned char;
using MTLRGBA = uchar4;
#else
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#if defined(__METAL__)
using MTLRGBA = half4;
using MTLColorBaseType = half;
#else
struct MTLRGBA
{
    _Float16 x, y, z, w;
};
using MTLColorBaseType = _Float16;
#endif
#else
using MTLColorBaseType = float;
using MTLRGBA = float4;
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif      // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

/*
 * @note TSim is the similarity type for volume in device memory.
 * @note TSimAcc is the similarity accumulation type for volume in device memory.
 * @note TSimRefine is the similarity type for volume refinement in device memory.
 */

#ifdef TSIM_USE_FLOAT
using TSim = float;
using TSimAcc = float;
#else
using TSim = unsigned char;
using TSimAcc = unsigned int;  // TSimAcc is the similarity accumulation type
#endif

#ifdef TSIM_REFINE_USE_HALF
#if defined(__METAL__)
using TSimRefine = half;
#else
using TSimRefine = _Float16;
#endif
#else
using TSimRefine = float;
#endif

#if defined(__METAL__)
#define make_float2(x, y) float2(x, y)
#define make_float3(x, y, z) float3(x, y, z)
#define make_float4(x, y, z, w) float4(x, y, z, w)
#define make_int3(x, y, z) int3(x, y, z)
#define make_uchar4(x, y, z, w) uchar4(x, y, z, w)
#endif

#if defined(__METAL__)
#define MTLDeviceAddrSpace device
#define MTLThreadAddrSpace thread
#define MTLConstantAddrSpace constant
#else
#define MTLDeviceAddrSpace
#define MTLThreadAddrSpace
#define MTLConstantAddrSpace
#endif

}
}
