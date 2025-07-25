// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#include <string>

namespace aliceVision {
namespace gpu {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

/**
 * @brief Check if the system support CUDA with the given parameters
 * @param[in] minComputeCapabilityMajor The minimum compute capability major
 * @param[in] minComputeCapabilityMinor The minimum compute capability minor
 * @param[in] minTotalDeviceMemory The minimum device total memory in MB
 * @return True if system support CUDA with the given parameters
 */
bool gpuSupportCUDA(int minComputeCapabilityMajor, int minComputeCapabilityMinor, int minTotalDeviceMemory = 0);

/**
 * @brief gpuInformationCUDA
 * @return string with all CUDA device(s) information
 */
std::string gpuInformationCUDA();

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

/**
 * @brief Check if the system support Vulkan with the given parameters
 * @param[in] minTotalDeviceMemory The minimum device total memory in MB
 * @return True if system support Vulkan with the given parameters
 */
bool gpuSupportVulkan(int minTotalDeviceMemory = 0);

/**
 * @brief gpuInformationVulkan
 * @return string with all Vulkan device(s) information
 */
std::string gpuInformationVulkan();

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

/**
 * @brief Check if the system supports Metal with the given parameters
 * @param[in] minTotalDeviceMemory The minimum device total memory in MB
 * @return True if system supports Metal with the given parameters
 */
bool gpuSupportMTL(int minTotalDeviceMemory = 0);

/**
 * @brief gpuInformationMTL
 * @return string with all Metal device(s) information
 */
std::string gpuInformationMTL();

#else
    #error Unsupported backend for Depth Map calculations!
#endif

}  // namespace gpu
}  // namespace aliceVision
