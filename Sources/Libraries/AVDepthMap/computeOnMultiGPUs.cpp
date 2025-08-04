// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/computeOnMultiGPUs.hpp>

#include <AV/omp.hpp>
#include <AVDepthMap/Metal/host/utils.hpp>
#include <AVDepthMap/Metal/host/DeviceManager.hpp>

namespace aliceVision {
namespace depthMap {

void computeOnMultiGPUs(const std::vector<int>& cams, IGPUJob& gpujob, int nbGPUsToUse)
{
    const unsigned int nbGPUDevices = listMTLDevices();
    const unsigned int nbCPUThreads = omp_get_max_threads();

    ALICEVISION_LOG_INFO("Number of GPU devices: " << nbGPUDevices << ", number of CPU threads: " << nbCPUThreads);

    unsigned int nbThreads = std::min(nbGPUDevices, nbCPUThreads);

    if (nbGPUsToUse > 0)
    {
        // Use the user specified limit on the number of GPUs to use
        nbThreads = std::min(nbThreads, static_cast<unsigned int>(nbGPUsToUse));
    }

    if (nbThreads == 1)
    {
        // the GPU sorting is determined by an environment variable named CUDA_DEVICE_ORDER
        // possible values: FASTEST_FIRST (default) or PCI_BUS_ID
        const uint64_t prioDeviceID = DeviceManager::getInstance().getPriorityDeviceID();
        gpujob.compute(prioDeviceID, cams);
    }
    else
    {
        // backup max threads to keep potentially previously set value
        int previous_count_threads = omp_get_max_threads();
        omp_set_num_threads(nbThreads);  // create as many CPU threads as there are CUDA devices
        const std::vector<uint64_t>& deviceIDs = DeviceManager::getInstance().getDeviceIDs();
#pragma omp parallel for
        for(uint64_t i=0; i < nbThreads; i++)
        {
            const int cpuThreadId = omp_get_thread_num();
            const uint64_t mtlDeviceID = deviceIDs[i];

            ALICEVISION_LOG_INFO("CPU thread " << cpuThreadId << " (of " << nbThreads << ") uses Metal device: " << mtlDeviceID);

            const int nbCamsPerThread = (cams.size() / nbThreads);
            const int rcFrom = (cpuThreadId % nbThreads) * nbCamsPerThread;
            int rcTo = ((cpuThreadId % nbThreads) + 1) * nbCamsPerThread;
            if ((cpuThreadId % nbThreads) == nbThreads - 1)
            {
                rcTo = cams.size();
            }

            std::vector<int> subcams;
            subcams.reserve(cams.size());

            for (int rc = rcFrom; rc < rcTo; ++rc)
            {
                subcams.push_back(cams[rc]);
            }

            gpujob.compute(mtlDeviceID, subcams);
        }
        omp_set_num_threads(previous_count_threads);
    }
}

}  // namespace depthMap
}  // namespace aliceVision
