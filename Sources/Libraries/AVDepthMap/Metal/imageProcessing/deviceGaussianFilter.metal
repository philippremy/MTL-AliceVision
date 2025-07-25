// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/BufPtr.hpp>
#include <AVDepthMap/Metal/device/buffer.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>
#include <AVDepthMap/Metal/util/MTLMipmappedTexture.hpp>
#include <AVDepthMap/Metal/device/DeviceGaussianFilter.hpp>

namespace aliceVision {
namespace depthMap {

/*********************************************************************************
 * device functions definitions
 *********************************************************************************/

void mtl_swap_float(thread float& a, thread float& b)
{
    float temp = a;
    a = b;
    b = temp;
}
    
void mtl_swap_float(device float& a, device float& b)
{
    float temp = a;
    a = b;
    b = temp;
}

/*********************************************************************************
 * kernel definitions
 *********************************************************************************/

/*
 * @note This kernel implementation is not optimized because the Gaussian filter is separable.
 */
kernel void downscaleWithGaussianBlur_kernel(device MTLRGBA* in_img_tex [[buffer(0)]], // As per the CUDA implementation, this uses CudaRGBA, which does not use normalized coordinates but might use interpolation
                                             device MTLRGBA* out_downscaledImg_d [[buffer(1)]],
                                             constant int* d_gaussianArrayOffset [[buffer(2)]],
                                             constant float* d_gaussianArray [[buffer(3)]],
                                             constant const downscaleWithGaussianBlur_kernel_PC& kArgs [[buffer(30)]],
                                             const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if((x < kArgs.downscaledImgWidth) && (y < kArgs.downscaledImgHeight))
    {
        const float s = float(kArgs.downscale) * 0.5f;

        float4 accPix = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sumFactor = 0.0f;
        
        // Create Mipmapped texture wrapper
        MTLMipmappedTexture<MTLRGBA> in_img_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(in_img_tex, kArgs.in_img_texLevel0Width, kArgs.in_img_texLevel0Height, kArgs.in_img_texLevelCount);

        for(int i = -kArgs.gaussRadius; i <= kArgs.gaussRadius; i++)
        {
            for(int j = -kArgs.gaussRadius; j <= kArgs.gaussRadius; j++)
            {
                const float4 curPix = tex2D_float4(in_img_tex_wrapper, float(x * kArgs.downscale + j) + s, float(y * kArgs.downscale + i) + s);
                const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, kArgs.downscale - 1, i + kArgs.gaussRadius) *
                                     getGauss(d_gaussianArrayOffset, d_gaussianArray, kArgs.downscale - 1, j + kArgs.gaussRadius); // domain factor

                accPix = accPix + curPix * factor;
                sumFactor += factor;
            }
        }

        device MTLRGBA& out = BufPtr<MTLRGBA, MetalAddressSpace::Device>(out_downscaledImg_d, kArgs.out_downscaledImg_p).at(size_t(x), size_t(y));
        out.x = accPix.x / sumFactor;
        out.y = accPix.y / sumFactor;
        out.z = accPix.z / sumFactor;
        out.w = accPix.w / sumFactor;
    }
}
    

kernel void gaussianBlurVolumeZ_kernel(device float* out_volume_d [[buffer(0)]],
                                       device const float* in_volume_d [[buffer(1)]],
                                       constant int* d_gaussianArrayOffset [[buffer(2)]],
                                       constant float* d_gaussianArray [[buffer(3)]],
                                       constant const gaussianBlurVolumeZ_kernel_PC& kArgs [[buffer(30)]],
                                       const uint3 gid [[thread_position_in_grid]],
                                       const uint3 bid [[threadgroup_position_in_grid]])
{
    const int vx = gid.x;
    const int vy = gid.y;
    const int vz = bid.z;

    const int gaussScale = kArgs.gaussRadius - 1;

    if(vx >= kArgs.volDimX || vy >= kArgs.volDimY)
        return;

    float sum = 0.0f;
    float sumFactor = 0.0f;

    for(int rz = -kArgs.gaussRadius; rz <= kArgs.gaussRadius; rz++)
    {
        const int iz = vz + rz;
        if((iz < kArgs.volDimZ) && (iz > 0))
        {
            const float value = float(*get3DBufferAt(in_volume_d, kArgs.in_volume_s, kArgs.in_volume_p, vx, vy, iz));
            const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, gaussScale, rz + kArgs.gaussRadius);
            sum += value * factor;
            sumFactor += factor;
        }
    }

    *get3DBufferAt(out_volume_d, kArgs.out_volume_s, kArgs.out_volume_p, vx, vy, vz) = float(sum / sumFactor);
}

kernel void gaussianBlurVolumeXYZ_kernel(device float* out_volume_d [[buffer(0)]],
                                         device const float* in_volume_d [[buffer(1)]],
                                         constant int* d_gaussianArrayOffset [[buffer(2)]],
                                         constant float* d_gaussianArray [[buffer(3)]],
                                         constant const gaussianBlurVolumeXYZ_kernel_PC& kArgs [[buffer(30)]],
                                         const uint3 gid [[thread_position_in_grid]],
                                         const uint3 bid [[threadgroup_position_in_grid]])
{
    const int vx = gid.x;
    const int vy = gid.y;
    const int vz = bid.z;

    const int gaussScale = kArgs.gaussRadius - 1;

    if(vx >= kArgs.volDimX || vy >= kArgs.volDimY)
        return;

    const int xMinRadius = max(-kArgs.gaussRadius, -vx);
    const int yMinRadius = max(-kArgs.gaussRadius, -vy);
    const int zMinRadius = max(-kArgs.gaussRadius, -vz);

    const int xMaxRadius = min(kArgs.gaussRadius, kArgs.volDimX - vx - 1);
    const int yMaxRadius = min(kArgs.gaussRadius, kArgs.volDimY - vy - 1);
    const int zMaxRadius = min(kArgs.gaussRadius, kArgs.volDimZ - vz - 1);

    float sum = 0.0f;
    float sumFactor = 0.0f;

    for(int rx = xMinRadius; rx <= xMaxRadius; rx++)
    {
        const int ix = vx + rx;

        for(int ry = yMinRadius; ry <= yMaxRadius; ry++)
        {
            const int iy = vy + ry;

            for(int rz = zMinRadius; rz <= zMaxRadius; rz++)
            {
                const int iz = vz + rz;
   
                const float value = float(*get3DBufferAt(in_volume_d, kArgs.in_volume_s, kArgs.in_volume_p, ix, iy, iz));
                const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, gaussScale, rx + kArgs.gaussRadius) * getGauss(d_gaussianArrayOffset, d_gaussianArray, gaussScale, ry + kArgs.gaussRadius) * getGauss(d_gaussianArrayOffset, d_gaussianArray, gaussScale, rz + kArgs.gaussRadius);
                sum += value * factor;
                sumFactor += factor;
            }
        }
    }

    *get3DBufferAt(out_volume_d, kArgs.out_volume_s, kArgs.out_volume_p, vx, vy, vz) = float(sum / sumFactor);
}
 
/**
 * @warning: use an hardcoded buffer size, so max radius value is 3.
 */
kernel void medianFilter3_kernel(device float* tex [[buffer(0)]], device float* texLab_d [[buffer(1)]], constant const medianFilter3_kernel_PC& kArgs, const uint3 gid [[thread_position_in_grid]])
{
    const int radius = 3;
    const int x = gid.x;
    const int y = gid.y;

    if((x >= kArgs.width - radius) || (y >= kArgs.height - radius) || (x < radius) || (y < radius))
        return;

    const int filterWidth = radius * 2 + 1;
    const int filterNbPixels = filterWidth * filterWidth;

    // VSA ??
    float buf[filterNbPixels]; // filterNbPixels

    // Create Mipmap Texture Wrapper
    // NOTE: Not an actually used kernel. Assuming that it uses non-normalized coordinates and bilinear filtering
    MTLMipmappedTexture<float> tex_wrapper = MTLMipmappedTexture<float>(tex, kArgs.texLevel0Width, kArgs.texLevel0Height, kArgs.texLevelCount);
    
    // Assign masked values to buf
    for(int yi = 0; yi < filterWidth; ++yi)
    {
        for(int xi = 0; xi < filterWidth; ++xi)
        {
            float pix = tex_wrapper.tex2DLod<FilterMode::Bilinear>(x + xi - radius, y + yi - radius);
            buf[yi * filterWidth + xi] = pix;
        }
    }

    // Calculate until we get the median value
    for(int k = 0; k < filterNbPixels; ++k) // (filterNbPixels + 1) / 2
        for(int l = 0; l < filterNbPixels; ++l)
            if(buf[k] < buf[l])
                mtl_swap_float(buf[k], buf[l]);

    BufPtr<float, MetalAddressSpace::Device>(texLab_d, kArgs.texLab_p).at(x, y) = buf[radius * filterWidth + radius];
}

} // namespace depthMap
} // namespace aliceVision

