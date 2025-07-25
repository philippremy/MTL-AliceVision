// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVGPU/Vulkan/memory.hpp>
#include <AVDepthMap/Vulkan/Types.hpp>

// maximum number of patch pattern subparts
// note: each patch pattern subpart gives one similarity
#define ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS 4

// maximum number of coordinates per patch pattern subpart
#define ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS 24

namespace aliceVision {
namespace depthMap {

/**
 * @struct DevicePatchPatternSubpart
 *
 * @brief Support class to maintain a subpart of a patch pattern in gpu constant memory.
 *        Each patch pattern subpart gives one similarity score.
 *
 * @note Should be entirely initialize from host memory.
 *       CUDA doesn't support default initialization for struct in constant memory.
 */
struct DevicePatchPatternSubpart
{
    int nbCoordinates;                                                     //< subpart number of coordinate
    float level;                                                           //< subpart related mipmap level (>=0)
    float downscale;                                                       //< subpart related mipmap downscale (>=1)
    float weight;                                                          //< subpart related similarity weight in range (0, 1)
    bool isCircle;                                                         //< subpart is a circle
    int wsh;                                                               //< subpart half-width (full and circle)
    float2 coordinates[ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS];  //< subpart coordinate list
};

/**
 * @struct DevicePatchPattern
 * @brief Support class to maintain a patch pattern in gpu constant memory.
 */
struct DevicePatchPattern
{
    int nbSubparts;                                                             //< patch pattern number of subparts (>0)
    DevicePatchPatternSubpart subparts[ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS];  //< patch pattern subparts (one similarity per subpart)
};

class DevicePatchPatternConstant
{
public:
    DevicePatchPatternConstant(const DevicePatchPatternConstant&) = delete;
    DevicePatchPatternConstant& operator=(const DevicePatchPatternConstant&) = delete;

    static DevicePatchPatternConstant* getInstance(uint32_t deviceID);

    const VulkanBuffer<DevicePatchPattern>& getPatchPatternConstant() const;

private:
    explicit DevicePatchPatternConstant(uint32_t deviceID);

    static std::unordered_map<uint32_t, DevicePatchPatternConstant*> m_these;
    static std::mutex m_initMutex;

    VulkanBuffer<DevicePatchPattern> m_constantPatchPattern;
};

}  // namespace depthMap
}  // namespace aliceVision
