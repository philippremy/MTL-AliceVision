// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Vulkan/DeviceGaussianArray.hpp>

namespace aliceVision {
namespace depthMap {

DeviceGaussianArray::DeviceGaussianArray(uint32_t deviceID)
    : m_gaussianArrayOffsets(
        VulkanMemoryBase<int>::createBuffer(
            deviceID,
            MAX_CONSTANT_GAUSS_SCALES,
            false,
            false)),
    m_gaussianArray(VulkanMemoryBase<float>::createBuffer(
            deviceID,
            MAX_CONSTANT_GAUSS_MEM_SIZE,
            false,
            false)),
    m_deviceID(deviceID)
{
    // TODO: Fille Gaussian Array?
}

DeviceGaussianArray* DeviceGaussianArray::getInstance(uint32_t deviceID)
{
    std::lock_guard lock(m_initMutex);
    auto it = m_these.find(deviceID);
    if (it != m_these.end()) {
        return it->second;
    }

    // Create a new instance and store it per deviceID
    DeviceGaussianArray* newInstance = new DeviceGaussianArray(deviceID);
    m_these[deviceID] = newInstance;

    return newInstance;
}

const VulkanBuffer<int>& DeviceGaussianArray::getGaussianArrayOffsetBuffer() const
{
    return this->m_gaussianArrayOffsets;
}

const VulkanBuffer<float>& DeviceGaussianArray::getGaussianArrayBuffer() const
{
    return this->m_gaussianArray;
}

void DeviceGaussianArray::createConstantGaussianArray(int scales)
{
    if(scales >= MAX_CONSTANT_GAUSS_SCALES)
    {
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    int h_gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES] = {};
    float h_gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE] = {0.0f};

    int sumSizes = 0;

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        h_gaussianArrayOffset[scale] = sumSizes;
        const int radius = scale + 1;
        const int size = 2 * radius + 1;
        sumSizes += size;
    }

    if(sumSizes >= MAX_CONSTANT_GAUSS_MEM_SIZE)
    {
        throw std::runtime_error( "Programming error: too little memory allocated for "
            + std::to_string(MAX_CONSTANT_GAUSS_SCALES) + " Gaussian kernels. Enlarge and recompile." );
    }

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        const int radius = scale + 1;
        const float delta  = 1.0f;
        const int size   = 2 * radius + 1;

        for(int idx = 0; idx < size; idx++)
        {
            int x = idx - radius;
            h_gaussianArray[h_gaussianArrayOffset[scale]+idx] = expf(-(x * x) / (2 * delta * delta));
        }
    }

    this->m_gaussianArrayOffsets = VulkanMemoryBase<int>::createBuffer(
        m_deviceID,
        MAX_CONSTANT_GAUSS_SCALES,
        false,
        false);

    this->m_gaussianArrayOffsets.copyFromHostToBuffer(h_gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES, 0, 0);

    this->m_gaussianArray = VulkanMemoryBase<float>::createBuffer(
        m_deviceID,
        MAX_CONSTANT_GAUSS_MEM_SIZE,
        false,
        false
    );

    this->m_gaussianArray.copyFromHostToBuffer(h_gaussianArray, MAX_CONSTANT_GAUSS_MEM_SIZE, 0, 0);

}

std::mutex DeviceGaussianArray::m_initMutex;
std::unordered_map<uint32_t, DeviceGaussianArray*> DeviceGaussianArray::m_these = {};

}
}