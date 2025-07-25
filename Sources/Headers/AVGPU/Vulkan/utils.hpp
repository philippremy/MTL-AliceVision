// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <AVGPU/Vulkan/device.hpp>

inline uint64_t getAvailableMemory(uint32_t deviceID)
{

    const auto physDev = VulkanManager::getInstance()->m_physDevs.at(deviceID);
    const auto physDevMemProps = physDev->getMemoryProperties();
    uint64_t memSum = 0;
    for(const auto& memHeap : physDevMemProps.memoryHeaps) {
        memSum += memHeap.size;
    }
    return memSum / (1024 * 1024);
}