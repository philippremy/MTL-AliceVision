// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/ComputeOnMultiGPUs.hpp>

#include <AV/omp.hpp>
#include <AVGPU/Vulkan/device.hpp>

#include <ranges>

namespace aliceVision {
namespace depthMap {

void computeOnMultiGPUs(const std::vector<int>& cams, IGPUJob& gpujob, int nbGPUsToUse)
{
    const auto& nbGPUDevices = VulkanManager::getInstance()->m_devs;
    const int nbCPUThreads = omp_get_max_threads();

    ALICEVISION_LOG_INFO("Number of GPU devices: " << nbGPUDevices.size() << ", number of CPU threads: " << nbCPUThreads);

    int nbThreads = std::min(static_cast<int>(nbGPUDevices.size()), nbCPUThreads);

    if (nbGPUsToUse > 0)
    {
        // Use the user specified limit on the number of GPUs to use
        nbThreads = std::min(nbThreads, nbGPUsToUse);
    }

    if (nbThreads == 1)
    {
        // The device is selected as indicated in getPriorityDeviceID().
        gpujob.compute(VulkanManager::getInstance()->getPriorityDeviceID(), cams);
    }
    else
    {
        // Create an ordered list of deviceIDs
        auto const& keys = std::views::keys(nbGPUDevices);
        std::vector<uint32_t> deviceIDs = std::vector(keys.begin(), keys.end());
        // backup max threads to keep potentially previously set value
        int previous_count_threads = omp_get_max_threads();
        omp_set_num_threads(nbThreads);  // create as many CPU threads as there are CUDA devices
#pragma omp parallel
        {
            const int cpuThreadId = omp_get_thread_num();
            const int deviceId = deviceIDs[cpuThreadId];

            ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses CUDA device: " << deviceId);

            const int nbCamsPerThread = (cams.size() / nbThreads);
            const int rcFrom = cpuThreadId * nbCamsPerThread;
            int rcTo = (cpuThreadId + 1) * nbCamsPerThread;
            if (deviceId == nbThreads - 1)
            {
                rcTo = cams.size();
            }

            std::vector<int> subcams;
            subcams.reserve(cams.size());

            for (int rc = rcFrom; rc < rcTo; ++rc)
            {
                subcams.push_back(cams[rc]);
            }

            gpujob.compute(deviceId, subcams);
        }
        omp_set_num_threads(previous_count_threads);
    }
}

}  // namespace depthMap
}  // namespace aliceVision
