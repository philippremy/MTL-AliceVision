// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/CustomPatchPatternParams.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Build user custom patch pattern in Vulkan memory.
 * @param[in] deviceID The deviceID for which the PatchPatternParams memory
 * should be created
 * @param[in] patchParams the user custom patch pattern parameters
 */
void buildCustomPatchPattern(uint32_t deviceID, const CustomPatchPatternParams& patchParams);

}  // namespace depthMap
}  // namespace aliceVision
