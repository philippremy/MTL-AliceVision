// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Metal/Metal.hpp>

#include <AVDepthMap/Metal/host/DeviceCommandManager.hpp>

#include <map>
#include <vector>

namespace aliceVision {
namespace depthMap {

class DeviceManager
{
public:
    static DeviceManager& getInstance();

    const std::vector<uint64_t>& getDeviceIDs() const;
    const uint64_t getPriorityDeviceID() const;
    const std::unordered_map<uint64_t, NS::SharedPtr<MTL::Device>>& getDeviceMap() const;
    const NS::SharedPtr<MTL::Device>& getDevice(uint64_t forDeviceID) const;
    const NS::SharedPtr<MTL::CommandQueue>& getCommandQueue(uint64_t forDeviceID) const;
    MTLCommandManager* getCommandManager(uint64_t forDeviceID);

    // no copy constructor
    DeviceManager(DeviceManager const&) = delete;

    // no implicit assignment constructor
    DeviceManager& operator=(DeviceManager const&) = delete;

private:
    DeviceManager();
    ~DeviceManager() = default;

    static DeviceManager* _instance;

    std::vector<uint64_t> _deviceIDs;
    std::unordered_map<uint64_t, NS::SharedPtr<MTL::Device>> _deviceMap;
    std::unordered_map<uint64_t, NS::SharedPtr<MTL::CommandQueue>> _commandQueueMap;
    std::unordered_map<uint64_t, MTLCommandManager> _commandManagerMap;
};

}
}
