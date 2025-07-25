// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Vulkan/DevicePatchPattern.hpp>

namespace aliceVision {
namespace depthMap {

DevicePatchPatternConstant::DevicePatchPatternConstant(uint32_t deviceID)
    : m_constantPatchPattern(
        VulkanMemoryBase<DevicePatchPattern>::createBuffer(
            deviceID,
            1,
            false,
            false))
{
    // TODO: Fill Patch Pattern?
}

DevicePatchPatternConstant *DevicePatchPatternConstant::getInstance(uint32_t deviceID)
{
    std::lock_guard lock(m_initMutex);
    auto it = m_these.find(deviceID);
    if (it != m_these.end()) {
        return it->second;
    }

    // Create a new instance and store it per deviceID
    DevicePatchPatternConstant* newInstance = new DevicePatchPatternConstant(deviceID);
    m_these[deviceID] = newInstance;

    return newInstance;
}

const VulkanBuffer<DevicePatchPattern>& DevicePatchPatternConstant::getPatchPatternConstant() const
{
    return this->m_constantPatchPattern;
}

std::mutex DevicePatchPatternConstant::m_initMutex;
std::unordered_map<uint32_t, DevicePatchPatternConstant*> DevicePatchPatternConstant::m_these = {};

}
}