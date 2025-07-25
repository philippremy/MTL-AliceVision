// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVGPU/Metal/command.hpp>
#include <AVGPU/Metal/device.hpp>

#include <sys/sysctl.h>

namespace aliceVision {
namespace gpu {

MTLDeviceManager::MTLDeviceManager()
{
    // Get all devices
    NS::Array* devices = MTL::CopyAllDevices();

    // Store devices
    for (uint i = 0; i < devices->count(); i++)
    {
        NS::SharedPtr<MTL::Device> device = NS::RetainPtr(devices->object<MTL::Device>(i));
        this->m_devices.emplace(device->registryID(), device);
    }

    // Store queues
    for (uint i = 0; i < devices->count(); i++)
    {
        NS::SharedPtr<MTL::CommandQueue> cmdQueue = NS::TransferPtr(devices->object<MTL::Device>(i)->newCommandQueue());
        this->m_queues.emplace(devices->object<MTL::Device>(i)->registryID(), cmdQueue);
    }

    // Create Resource Managers for every device
    for (uint i = 0; i < devices->count(); i++)
    {
        const MTLResourceManager resourceManager = MTLResourceManager(this->m_devices.at(devices->object<MTL::Device>(i)->registryID()));
        this->m_resourceManagers.emplace(devices->object<MTL::Device>(i)->registryID(), std::make_unique<MTLResourceManager>(resourceManager));
    }

    // Create Resource Managers for every device
    for (uint i = 0; i < devices->count(); i++)
    {
        const MTLCommandManager commandManager = MTLCommandManager(this->m_devices.at(devices->object<MTL::Device>(i)->registryID()));
        this->m_commandManagers.emplace(devices->object<MTL::Device>(i)->registryID(), std::make_unique<MTLCommandManager>(commandManager));
    }

    // Release device array
    devices->release();
}

MTLDeviceManager* MTLDeviceManager::getInstance()
{
    // Initialize the static member
    if (m_this == nullptr)
    {
        std::lock_guard lock(m_initMutex);
        if (m_this == nullptr)
        {
            m_this = new MTLDeviceManager();
        }
    }
    return m_this;
}

const std::unordered_map<uint64_t, NS::SharedPtr<MTL::Device>>& MTLDeviceManager::getDevices() const { return this->m_devices; }

NS::SharedPtr<MTL::Device> MTLDeviceManager::getDevice(const uint64_t deviceID) const { return this->m_devices.at(deviceID); }

NS::SharedPtr<MTL::CommandQueue> MTLDeviceManager::getQueue(const uint64_t deviceID) const { return this->m_queues.at(deviceID); }

MTLResourceManager* MTLDeviceManager::getResourceManager(const uint64_t deviceID) const { return this->m_resourceManagers.at(deviceID).get(); }

MTLCommandManager* MTLDeviceManager::getCommandManager(const uint64_t deviceID) const { return this->m_commandManagers.at(deviceID).get(); }

const uint64_t MTLDeviceManager::getPriorityDeviceID() const
{
    uint64_t priorityDeviceID = UINT64_MAX;
    uint64_t previousMaxRAMSize = 0;

    for (const auto [deviceID, device] : this->m_devices)
    {
        if (!device->lowPower() && device->recommendedMaxWorkingSetSize() >= 3.221e+9)
        {
            return deviceID;
        }
        if (previousMaxRAMSize < device->recommendedMaxWorkingSetSize())
        {
            previousMaxRAMSize = device->recommendedMaxWorkingSetSize();
            priorityDeviceID = deviceID;
        }
    }

    if (priorityDeviceID == UINT64_MAX)
    {
        ALICEVISION_THROW_ERROR("No Metal device found! This is most likely a bug, please submit a bug report!");
    }

    return priorityDeviceID;
}

void MTLDeviceManager::getDeviceMemoryInfo(const uint64_t deviceID, double& availableMB, double& usedMB, double& totalMB) const
{
    // Caluclate total memory based on being unified
    uint64_t host_memsize = 0;
    size_t size = sizeof(host_memsize);
    if (sysctlbyname("hw.memsize", &host_memsize, &size, nullptr, 0) != KERN_SUCCESS) {
        host_memsize = 0;
    }
    const auto device = this->m_devices.at(deviceID);
    const uint64_t totalMem = device->hasUnifiedMemory() ? host_memsize : device->recommendedMaxWorkingSetSize() + host_memsize;
    const uint64_t usedMem = device->currentAllocatedSize();
    const int64_t availableMem = totalMem - usedMem;

    totalMB = totalMem / 1024.0 / 1024.0;
    usedMB = usedMem / 1024.0 / 1024.0;
    availableMB = availableMem / 1024.0 / 1024.0;
}

void MTLDeviceManager::logDeviceMemoryInfo(const uint64_t deviceID) const
{
    double availableMB, usedMB, totalMB;
    this->getDeviceMemoryInfo(deviceID, availableMB, usedMB, totalMB);
    ALICEVISION_LOG_INFO("Device memory (device id: " << deviceID << "):" << std::endl
                                                  << "\t- used: " << usedMB << " MB" << std::endl
                                                  << "\t- available: " << availableMB << " MB" << std::endl
                                                  << "\t- total: " << totalMB << " MB");
}

std::mutex MTLDeviceManager::m_initMutex;    // Mutex used for initialization
MTLDeviceManager* MTLDeviceManager::m_this;  // Static Member holding a pointer to self

MTLResourceManager::MTLResourceManager(NS::SharedPtr<MTL::Device> device)
  : m_device(device)
{}

}  // namespace gpu
}  // namespace aliceVision