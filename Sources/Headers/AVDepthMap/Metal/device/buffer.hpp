// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/BufPtr.hpp>
#include <AVDepthMap/Metal/util/MTLMipmappedTexture.hpp>

namespace aliceVision {
namespace depthMap {

/**
* @brief
* @param[int] ptr
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline device T* get2DBufferAt(device T* ptr, size_t pitch, size_t x, size_t y)
{
    return &(BufPtr<T, MetalAddressSpace::Device>(ptr,pitch).at(x,y));
}

/**
* @brief
* @param[int] ptr
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline constant T* get2DBufferAt(constant T* ptr, size_t pitch, size_t x, size_t y)
{
    return &(BufPtr<T, MetalAddressSpace::Constant>(ptr,pitch).at(x,y));
}

/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline device T* get3DBufferAt(device T* ptr, size_t spitch, size_t pitch, size_t x, size_t y, size_t z)
{
    return ((device T*)(((device char*)ptr) + z * spitch + y * pitch)) + x;
}

/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
inline constant T* get3DBufferAt(constant T* ptr, size_t spitch, size_t pitch, size_t x, size_t y, size_t z)
{
    return ((constant T*)(((constant char*)ptr) + z * spitch + y * pitch)) + x;
}

template <typename T>
inline device const T* get3DBufferAt(device const T* ptr, size_t spitch, size_t pitch, size_t x, size_t y, size_t z)
{
    return ((device const T*)(((device const char*)ptr) + z * spitch + y * pitch)) + x;
}

template <typename T>
inline device T* get3DBufferAt(device T* ptr, size_t spitch, size_t pitch, device const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

template <typename T>
inline device T* get3DBufferAt(device T* ptr, size_t spitch, size_t pitch, thread const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

template <typename T>
inline constant T* get3DBufferAt(constant T* ptr, size_t spitch, size_t pitch, constant const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

template <typename T>
inline device const T* get3DBufferAt(device const T* ptr, size_t spitch, size_t pitch, device const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

template <typename T>
inline device const T* get3DBufferAt(device const T* ptr, size_t spitch, size_t pitch, thread const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

inline float multi_fminf(float a, float b, float c)
{
  return fmin(fmin(a, b), c);
}

inline float multi_fminf(float a, float b, float c, float d)
{
  return fmin(fmin(fmin(a, b), c), d);
}

//#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
//
//__device__ inline float4 tex2D_float4(cudaTextureObject_t rc_tex, float x, float y)
//{
//#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
//    // cudaReadNormalizedFloat
//    float4 a = tex2D<float4>(rc_tex, x, y);
//    return make_float4(a.x * 255.0f, a.y * 255.0f, a.z * 255.0f, a.w * 255.0f);
//#else
//    // cudaReadElementType
//    uchar4 a = tex2D<uchar4>(rc_tex, x, y);
//    return make_float4(a.x, a.y, a.z, a.w);
//#endif
//}
//
//#else
//
//__device__ inline float4 tex2D_float4(cudaTextureObject_t rc_tex, float x, float y)
//{
//    return tex2D<float4>(rc_tex, x, y);
//}
//#endif

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

inline float4 tex2D_float4(MTLMipmappedTexture<MTLRGBA> rc_tex, float x, float y)
{
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
    // cudaReadNormalizedFloat
    uchar4 a = rc_tex.tex2DLod<FilterMode::Bilinear>(x, y);
    float4 a_tmp = float4(a.x / 255.f, a.y / 255.f, a.z / 255.f, a.w / 255.f);
    return make_float4(a_tmp.x * 255.0f, a_tmp.y * 255.0f, a_tmp.z * 255.0f, a_tmp.w * 255.0f);
#else
    // cudaReadElementType
    uchar4 a = rc_tex.tex2DLod<FilterMode::NearestNeighbor>(x, y);
    return make_float4(a.x, a.y, a.z, a.w);
#endif
}

#else

inline float4 tex2D_float4(MTLMipmappedTexture<MTLRGBA> rc_tex, float x, float y)
{
#if defined ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
    MTLRGBA temp = rc_tex.tex2DLod<FilterMode::Bilinear>(x, y);
#else
    MTLRGBA temp = rc_tex.tex2DLod<FilterMode::NearestNeighbor>(x, y);
#endif
    return float4(temp.x, temp.y, temp.z, temp.w);
}

#endif

} // namespace depthMap
} // namespace aliceVision


