// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Vulkan/DeviceCameraParams.hpp>

namespace aliceVision {
namespace depthMap {

DeviceCameraParamsArray::DeviceCameraParamsArray(uint32_t deviceID)
    : m_constantCameraParametersArray(
        VulkanMemoryBase<DeviceCameraParams>::createBuffer(
            deviceID,
            ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS,
            false,
            false))
{
    // TODO: Fill Camera Params?
}

DeviceCameraParamsArray *DeviceCameraParamsArray::getInstance(uint32_t deviceID)
{
    std::lock_guard lock(m_initMutex);
    auto it = m_these.find(deviceID);
    if (it != m_these.end()) {
        return it->second;
    }

    // Create a new instance and store it per deviceID
    DeviceCameraParamsArray* newInstance = new DeviceCameraParamsArray(deviceID);
    m_these[deviceID] = newInstance;

    return newInstance;
}

const VulkanBuffer<DeviceCameraParams>& DeviceCameraParamsArray::getCameraParamsArray() const
{
    return this->m_constantCameraParametersArray;
}

std::mutex DeviceCameraParamsArray::m_initMutex;
std::unordered_map<uint32_t, DeviceCameraParamsArray*> DeviceCameraParamsArray::m_these = {};

}
}