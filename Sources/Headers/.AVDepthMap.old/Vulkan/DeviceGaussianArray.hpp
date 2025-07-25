// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVGPU/Vulkan/memory.hpp>

#define MAX_CONSTANT_GAUSS_SCALES 10
#define MAX_CONSTANT_GAUSS_MEM_SIZE 128

namespace aliceVision {
namespace depthMap {

class DeviceGaussianArray
{
public:
    DeviceGaussianArray(const DeviceGaussianArray&) = delete;
    DeviceGaussianArray& operator=(const DeviceGaussianArray&) = delete;

    static DeviceGaussianArray* getInstance(uint32_t deviceID);

    const VulkanBuffer<int>& getGaussianArrayOffsetBuffer() const;
    const VulkanBuffer<float>& getGaussianArrayBuffer() const;

    void createConstantGaussianArray(int scales);

private:
    explicit DeviceGaussianArray(uint32_t deviceID);

    static std::unordered_map<uint32_t, DeviceGaussianArray*> m_these;
    static std::mutex m_initMutex;

    uint32_t m_deviceID;
    VulkanBuffer<int> m_gaussianArrayOffsets;
    VulkanBuffer<float> m_gaussianArray;
};

}
}