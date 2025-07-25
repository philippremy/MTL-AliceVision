// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/utils.hpp>
#include <AVDepthMap/Metal/host/DeviceManager.hpp>
#include <AVSystem/Logger.hpp>

#include <ranges>
#include <sys/sysctl.h>

namespace aliceVision {
namespace depthMap {

unsigned int listMTLDevices()
{
    unsigned int nbDevices = 0;  // number of CUDA GPUs

    const auto& deviceMap = DeviceManager::getInstance().getDeviceMap();

    nbDevices = deviceMap.size();

    if (nbDevices < 1)
    {
        ALICEVISION_LOG_ERROR("No Metal capable devices detected.");
        return 0;
    }

    // display CPU and GPU configuration
    std::stringstream s;
    int i = 0;
    for (const auto& device : std::views::values(deviceMap))
    {
        s << "\t- Device " << i << ": " << std::string(device->name()->utf8String()) << std::endl;
        i++;
    }
    ALICEVISION_LOG_DEBUG(nbDevices << " Metal devices found:" << std::endl << s.str());

    return nbDevices;
}

void logDeviceMemoryInfo(const uint64_t deviceID)
{
    double availableMB;
    double totalMB;
    double usedMB;

    getDeviceMemoryInfo(availableMB, usedMB, totalMB, deviceID);

    ALICEVISION_LOG_INFO("Device memory (device id: " << deviceID << "):" << std::endl
                                                      << "\t- used: " << usedMB << " MB" << std::endl
                                                      << "\t- available: " << availableMB << " MB" << std::endl
                                                      << "\t- total: " << totalMB << " MB");
}

void getDeviceMemoryInfo(double& availableMB, double& usedMB, double& totalMB, const uint64_t deviceID)
{
    size_t iused;
    size_t itotal;

    const MTL::Device* device = DeviceManager::getInstance().getDevice(deviceID);

    // On non-uniform-memory devices, we add the host memory as it is still fully usable my Metal
    uint64_t host_memsize = 0;
    size_t size = sizeof(host_memsize);
    if (sysctlbyname("hw.memsize", &host_memsize, &size, nullptr, 0) != KERN_SUCCESS) {
        host_memsize = 0;
    }
    itotal = device->hasUnifiedMemory() ? host_memsize : device->recommendedMaxWorkingSetSize() + host_memsize;
    iused = device->currentAllocatedSize();

    availableMB = double(itotal - iused) / (1024.0 * 1024.0);
    totalMB = double(itotal) / (1024.0 * 1024.0);
    usedMB = double(iused) / (1024.0 * 1024.0);
}

}  // namespace depthMap
}  // namespace aliceVision
