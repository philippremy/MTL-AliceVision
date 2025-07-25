// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION

#define TSIM_REFINE_USE_HALF

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
typedef struct VulkanRGBA
{
    unsigned char x, y, z, w;
} VulkanRGBA;
using VulkanBaseType = unsigned char;
#else
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#if __has_include(<stdfloat>)
    #if __STDCPP_FLOAT16_T__ == 1
    typedef struct VulkanRGBA
    {
        std::float16_t x, y, z, w;
    } VulkanRGBA;
    using VulkanBaseType = std::float16_t;
    #endif // __STDCPP_FLOAT16_T__ == 1
#endif // __has_include(<stdfloat>)
#if !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
    #if defined(__clang__) || defined(__GNUC__)
    typedef struct VulkanRGBA
    {
        _Float16 x, y, z, w;
    } VulkanRGBA;
    using VulkanBaseType = _Float16;
    #else
    #include <fp16.h>
    typedef struct VulkanRGBA
    {
        uint16_t x, y, z, w;
    } VulkanRGBA;
    using VulkanBaseType = uint16_t;
    #endif // defined(__clang__) || defined(__GNUC__)
#endif // !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
#else
typedef struct VulkanRGBA
{
    float x, y, z, w;
} VulkanRGBA;
using VulkanColorBaseType = float;
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

// Conversion functions
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#if __has_include(<stdfloat>)
#if __STDCPP_FLOAT16_T__ == 1
    inline std::float16_t float2half(const float& in)
    {
        return static_cast<std::float16_t>(in);
    }
#endif // __STDCPP_FLOAT16_T__ == 1
#endif // __has_include(<stdfloat>)
#if !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
#if defined(__clang__) || defined(__GNUC__)
    inline _Float16 float2half(const float& in)
    {
        return static_cast<_Float16>(in);
    }
#else
    #include <fp16.h>
    inline uint16_t float2half(const float& in)
    {
        return fp16_ieee_from_fp32_value(in);
    }
#endif // defined(__clang__) || defined(__GNUC__)
#endif // !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
#endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF

#ifdef TSIM_USE_FLOAT
using TSim = float;
using TSimAcc = float;
#else
using TSim = unsigned char;
using TSimAcc = uint32_t;
#endif  // TSIM_USE_FLOAT

#ifdef TSIM_REFINE_USE_HALF
#if __has_include(<stdfloat>)
#if __STDCPP_FLOAT16_T__ == 1
using TSimRefine = std::float16_t;
#endif // __STDCPP_FLOAT16_T__ == 1
#endif // __has_include(<stdfloat>)
#if !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
#if defined(__clang__) || defined(__GNUC__)
using TSimRefine = _Float16;
#else
#include <fp16.h>
using TSimRefine = uint16_t;
#endif // defined(__clang__) || defined(__GNUC__)
#endif // !defined(__STDCPP_FLOAT16_T__) || __STDCPP_FLOAT16_T__ != 1
#else
using TSimRefine = float;
#endif  // TSIM_REFINE_USE_HALF

typedef struct int3
{
    uint32_t x, y, z;
} int3;

typedef struct float2
{
    float x, y;
} float2;

typedef struct float3
{
    float x, y, z;
} float3;

typedef struct float4
{
    float x, y, z, w;
} float4;

}
}