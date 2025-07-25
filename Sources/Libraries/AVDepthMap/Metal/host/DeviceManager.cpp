// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/DeviceManager.hpp>
#include <AVSystem/Logger.hpp>

#include <ranges>

namespace aliceVision {
namespace depthMap {

DeviceManager::DeviceManager()
{
    NS::Array* devices = MTL::CopyAllDevices();
    for (uint64_t idx=0; idx < devices->count(); idx++)
    {
        MTL::Device* device = static_cast<MTL::Device*>(devices->object(idx));
        this->_deviceMap.emplace(device->registryID(), device);
        this->_deviceIDs.emplace_back(device->registryID());
    }
    devices->release();
    for (MTL::Device* device : std::views::values(this->_deviceMap))
    {
        this->_commandQueueMap.emplace(device->registryID(), device->newCommandQueue());
    }
        for (MTL::Device* device : std::views::values(this->_deviceMap))
    {
        this->_commandManagerMap.emplace(device->registryID(), MTLCommandManager(device));
    }
}

DeviceManager::~DeviceManager()
{
    for (MTL::CommandQueue* queue : std::views::values(this->_commandQueueMap))
    {
        queue->release();
    }
    for (MTL::Device* device : std::views::values(this->_deviceMap))
    {
        device->release();
    }
}

DeviceManager& DeviceManager::getInstance()
{
    if (_instance == nullptr)
        _instance = new DeviceManager();
    return *_instance;
}

const std::vector<uint64_t>& DeviceManager::getDeviceIDs() const
{
    return this->_deviceIDs;
}

const uint64_t DeviceManager::getPriorityDeviceID() const
{
    uint64_t priorityDeviceID = UINT64_MAX;
    uint64_t previousMaxRAMSize = 0;

    for (const auto [deviceID, device] : this->_deviceMap)
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

const std::unordered_map<uint64_t, MTL::Device*>& DeviceManager::getDeviceMap() const
{
    return this->_deviceMap;
}

MTL::Device* DeviceManager::getDevice(uint64_t forDeviceID) const
{
    return _deviceMap.at(forDeviceID);
}

MTL::CommandQueue* DeviceManager::getCommandQueue(uint64_t forDeviceID) const
{
    return _commandQueueMap.at(forDeviceID);
}

MTLCommandManager* DeviceManager::getCommandManager(uint64_t forDeviceID)
{
    return &_commandManagerMap.at(forDeviceID);
}


DeviceManager* DeviceManager::_instance = nullptr;

}
}