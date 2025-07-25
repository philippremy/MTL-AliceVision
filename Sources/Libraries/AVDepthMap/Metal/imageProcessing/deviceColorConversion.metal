// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/device/buffer.hpp>
#include <AVDepthMap/Metal/device/color.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

kernel void rgb2lab_kernel(device MTLRGBA* inout_img_d [[buffer(0)]],
                           constant const rgb2lab_kernel_PC& kArgs [[buffer(30)]],
                           const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if((x >= kArgs.width) || (y >= kArgs.height))
        return;

    // corresponding input CudaRGBA
    device MTLRGBA* rgb = get2DBufferAt(inout_img_d, kArgs.inout_img_p, x, y);

    // MTLRGBA (uchar4 or float4 or half4) in range (0, 255)
    // rgb2xyz needs RGB in range (0, 1)
    constexpr float d = 1 / 255.f;

    // compute output CIELAB
    // RGB(0, 255) to XYZ(0, 1) to CIELAB(0, 255)
    float3 flab = xyz2lab(rgb2xyz(make_float3(float(rgb->x) * d, float(rgb->y) * d, float(rgb->z) * d)));

    // write output CIELAB
    rgb->x = flab.x;
    rgb->y = flab.y;
    rgb->z = flab.z;
}

} // namespace depthMap
} // namespace aliceVision
