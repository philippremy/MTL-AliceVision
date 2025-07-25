// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/device/buffer.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>
#include <AVDepthMap/Metal/device/DeviceGaussianFilter.hpp>

namespace aliceVision {
namespace depthMap {

kernel void createMipmappedArrayLevel_kernel(device MTLRGBA* inout_mipmapped_array_d [[buffer(0)]], // As per the CUDA implementation, this uses normalized coordinates and bilinear filtering
                                             constant int* d_gaussianArrayOffset [[buffer(1)]],
                                             constant float* d_gaussianArray [[buffer(2)]],
                                             constant const createMipmappedArrayLevel_kernel_PC& kArgs [[buffer(30)]],
                                             const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if(x >= kArgs.width || y >= kArgs.height)
        return;

    const float px = 1.f / float(kArgs.width);
    const float py = 1.f / float(kArgs.height);

    MTLRGBA sumColor = MTLRGBA(0, 0, 0, 0);
    float sumFactor = 0.0f;

    // Create a Mipmapped texture wrapper
    MTLMipmappedTexture<MTLRGBA> inout_mipmapped_array_d_wrapper = MTLMipmappedTexture<MTLRGBA>(inout_mipmapped_array_d, kArgs.inout_mipmapped_array_dLevel0Width, kArgs.inout_mipmapped_array_dLevel0Height, kArgs.inout_mipmapped_array_dLevelCount);

    for(int i = -kArgs.TRadius; i <= kArgs.TRadius; i++)
    {
        for(int j = -kArgs.TRadius; j <= kArgs.TRadius; j++)
        {
            // domain factor
            const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, 1, i + kArgs.TRadius) * getGauss(d_gaussianArrayOffset, d_gaussianArray, 1, j + kArgs.TRadius);

            // normalized coordinates
            const float u = (x + j + 0.5f) * px;
            const float v = (y + i + 0.5f) * py;

            // current pixel color
            const MTLRGBA color = inout_mipmapped_array_d_wrapper.tex2DLodNorm<FilterMode::Bilinear>(u, v, kArgs.previousMipLevel);

            // sum color
            sumColor = sumColor + color * factor;

            // sum factor
            sumFactor += factor;
        }
    }

    const MTLRGBA color = sumColor / sumFactor;

    // Create Mipmap Texture Wrapper
    // Create a Mipmapped texture wrapper

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color to unsigned char
    MTLRGBA out;
    out.x = MTLColorBaseType(color.x);
    out.y = MTLColorBaseType(color.y);
    out.z = MTLColorBaseType(color.z);
    out.w = MTLColorBaseType(color.w);

    // write output color
    inout_mipmapped_array_d_wrapper.writeLod(out, x, y, kArgs.currentMipLevel);
#else // texture use float4 or half4
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    // convert color to half
    MTLRGBA out;
    out.x = MTLColorBaseType(color.x);
    out.y = MTLColorBaseType(color.y);
    out.z = MTLColorBaseType(color.z);
    out.w = MTLColorBaseType(color.w);

    // write output color
    inout_mipmapped_array_d_wrapper.writeLod(out, x, y, kArgs.currentMipLevel);
#else // texture use float4
     // write output color
    inout_mipmapped_array_d_wrapper.writeLod(out, x, y, kArgs.currentMipLevel);
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}

/*
__global__ void createMipmappedArrayLevel_kernel(cudaSurfaceObject_t out_currentLevel_surf,
                                                 cudaTextureObject_t in_previousLevel_tex,
                                                 unsigned int width,
                                                 unsigned int height)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    // corresponding texture normalized coordinates
    const float u = (x + 0.5f) / float(width);
    const float v = (y + 0.5f) / float(height);

    // corresponding color in previous level texture
    const float4 color = tex2D_float4(in_previousLevel_tex, u, v);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color to unsigned char
    CudaRGBA out;
    out.x = CudaColorBaseType(color.x);
    out.y = CudaColorBaseType(color.y);
    out.z = CudaColorBaseType(color.z);
    out.w = CudaColorBaseType(color.w);

    // write output color
    surf2Dwrite(out, out_currentLevel_surf, int(x * sizeof(CudaRGBA)), int(y));
#else // texture use float4 or half4
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    // convert color to half
    CudaRGBA out;
    out.x = __float2half(color.x);
    out.y = __float2half(color.y);
    out.z = __float2half(color.z);
    out.w = __float2half(color.w);

    // write output color
    // note: surf2Dwrite cannot write half directly
    surf2Dwrite(*(reinterpret_cast<ushort4*>(&(out))), out_currentLevel_surf, int(x * sizeof(ushort4)), int(y));
#else // texture use float4
     // write output color
    surf2Dwrite(color, out_currentLevel_surf, int(x * sizeof(float4)), int(y));
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}
*/

// NOTE: This kernel is apparently not used.
kernel void createMipmappedArrayDebugFlatImage_kernel(device MTLRGBA* out_flatImage_d [[buffer(0)]],
                                                      device MTLRGBA* in_mipmappedArray_tex [[buffer(1)]], // Assuming this used normalized coordinates and bilinear filtering
                                                      constant const createMipmappedArrayDebugFlatImage_kernel_PC& kArgs [[buffer(30)]],
                                                      const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if(y >= kArgs.firstLevelHeight)
        return;

    // set default color value
    MTLRGBA color = MTLRGBA(0, 0, 0, 0);
    
    // Create Mipmapped Texture Wrapper
    MTLMipmappedTexture<MTLRGBA> in_mipmappedArray_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(in_mipmappedArray_tex, kArgs.in_mipmappedArray_texLevel0Width, kArgs.in_mipmappedArray_texLevel0Height, kArgs.in_mipmappedArray_texLevelCount);
    
    if(x < kArgs.firstLevelWidth)
    {
        // level 0
        // corresponding texture normalized coordinates
        const float u = (x + 0.5f) / float(kArgs.firstLevelWidth);
        const float v = (y + 0.5f) / float(kArgs.firstLevelHeight);

        // set color value from mipmappedArray texture
        color = in_mipmappedArray_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(u, v, 0.f);
    }
    else
    {
        // level from global y coordinate
        const unsigned int level = int(log2(1.0 / (1.0 - (y / float(kArgs.firstLevelHeight))))) + 1;
        const unsigned int levelDownscale = pow(float(2), float(level));
        const unsigned int levelWidth  = kArgs.firstLevelWidth  / levelDownscale;
        const unsigned int levelHeight = kArgs.firstLevelHeight / levelDownscale;

        // corresponding level coordinates
        const float lx = x - kArgs.firstLevelWidth;
        const float ly = y % levelHeight;

        // corresponding texture normalized coordinates
        const float u = (lx + 0.5f) / float(levelWidth);
        const float v = (ly + 0.5f) / float(levelHeight);

        if(u <= 1.f && v <= 1.f && level < kArgs.levels)
        {
            // set color value from mipmappedArray texture
            color = in_mipmappedArray_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(u, v, float(level));
        }
    }

    // write output color
    device MTLRGBA* out_colorPtr = get2DBufferAt(out_flatImage_d, kArgs.out_flatImage_p, x, y);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color from (0, 1) to (0, 255)
    color.x *= 255.f;
    color.y *= 255.f;
    color.z *= 255.f;
    color.w *= 255.f;

    out_colorPtr->x = MTLColorBaseType(color.x);
    out_colorPtr->y = MTLColorBaseType(color.y);
    out_colorPtr->z = MTLColorBaseType(color.z);
    out_colorPtr->w = MTLColorBaseType(color.w);
#else // texture use float4 or half4
    // convert color from (0, 255) to (0, 1)
    color.x /= 255.f;
    color.y /= 255.f;
    color.z /= 255.f;
    color.w /= 255.f;
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    out_colorPtr->x = half(color.x);
    out_colorPtr->y = half(color.y);
    out_colorPtr->z = half(color.z);
    out_colorPtr->w = half(color.w);
#else // texture use float4
    *out_colorPtr = color;
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}

} // namespace depthMap
} // namespace aliceVision

