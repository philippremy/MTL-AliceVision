// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVGPU/Vulkan/memory.hpp>
#include <AVDepthMap/Vulkan/Types.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @struct DeviceCameraParams
 * @brief Support class to maintain useful camera parameters in gpu memory.
 */
struct DeviceCameraParams
{
    float P[12];
    float iP[9];
    float R[9];
    float iR[9];
    float K[9];
    float iK[9];
    float3 C;
    float3 XVect;
    float3 YVect;
    float3 ZVect;
};

// global / constant data structures

#define ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS 100

class DeviceCameraParamsArray
{
public:
    DeviceCameraParamsArray(const DeviceCameraParamsArray&) = delete;
    DeviceCameraParamsArray& operator=(const DeviceCameraParamsArray&) = delete;

    static DeviceCameraParamsArray* getInstance(uint32_t deviceID);

    const VulkanBuffer<DeviceCameraParams>& getCameraParamsArray() const;

private:
    explicit DeviceCameraParamsArray(uint32_t deviceID);

    static std::unordered_map<uint32_t, DeviceCameraParamsArray*> m_these;
    static std::mutex m_initMutex;

    VulkanBuffer<DeviceCameraParams> m_constantCameraParametersArray;
};

}  // namespace depthMap
}  // namespace aliceVision
