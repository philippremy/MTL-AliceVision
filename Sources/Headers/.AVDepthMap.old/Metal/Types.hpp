//
//  SharedMetalTypes.h
//  AVDepthMapMetalKernels
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef SharedMetalTypes_h
#define SharedMetalTypes_h

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#define HALF_t half
#define HALF4_t half4
#endif

#if !defined(__METAL__)
#include <simd/vector.h>
using namespace simd;
#include <Metal/Metal.hpp>
#define HALF_t _Float16
#define HALF4_t __attribute__((__ext_vector_type__(4))) _Float16;
#endif

namespace aliceVision {
namespace depthMap {

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION

#define TSIM_REFINE_USE_HALF

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
using MTLRGBA = uchar4;
using MTLPixelBaseType = ushort;
#if !defined(__METAL__)
#define MTLPixelBaseTypeFormat MTL::PixelFormatR16Uint
#endif
#else
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
using MTLRGBA = HALF4_t;
using MTLPixelBaseType = HALF_t;
#if !defined(__METAL__)
#define MTLPixelBaseTypeFormat MTL::PixelFormatR16Float
#endif
#else
using MTLRGBA = float;
using MTLPixelBaseType = float;
#if !defined(__METAL__)
#define MTLPixelBaseTypeFormat MTL::PixelFormatR32Float
#endif
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

#ifdef TSIM_USE_FLOAT
using TSim = float;
using TSimAcc = float;
#if !defined(__METAL__)
#define MTLTSimFormat MTL::PixelFormatR32Float
#define MTLTSimAccFormat MTL::PixelFormatR32Float
#endif
#else
using TSim = ushort;
using TSimAcc = uint32_t;
#if !defined(__METAL__)
#define MTLTSimPixelFormat MTL::PixelFormatR16Uint
#define MTLTSimAccPixelFormat MTL::PixelFormatR32Uint
#endif
#endif  // TSIM_USE_FLOAT

#ifdef TSIM_REFINE_USE_HALF
using TSimRefine = HALF_t;
#if !defined(__METAL__)
#define MTLTSimRefineFormat MTL::PixelFormatR16Float
#endif
#else
using TSimRefine = float;
#if !defined(__METAL__)
#define MTLTSimRefineFormat MTL::PixelFormatR32Float
#endif
#endif  // TSIM_REFINE_USE_HALF

}
}

#endif /* SharedMetalTypes_h */