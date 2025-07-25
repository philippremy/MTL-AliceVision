// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/host/memory.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief In-place color conversion from RGB into CIELAB using CUDA.
 * @param[in, out] inout_img_dmp the input RGB buffer, the output LAB buffer in device memory
 * @param[in] stream the CUDA stream for gpu execution
 */
void mtl_rgb2lab(MTLDeviceMemoryPitched<MTLRGBA, 2>& inout_img_dmp, uint64_t deviceID);

}  // namespace depthMap
}  // namespace aliceVision
