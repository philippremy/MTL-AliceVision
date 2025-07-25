// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Get and log available Metal devices.
 * @return the number of Metal devices
 */
unsigned int listMTLDevices();

/**
 * @brief Log current Metal device memory information.
 */
void logDeviceMemoryInfo(uint64_t deviceID);

/**
 * @brief Get current Metal device memory information.
 * @param[out] availableMB the available memory in MB on the current Metal device
 * @param[out] usedMB the used memory in MB on the current Metal device
 * @param[out] totalMB the total memory in MB on the current Metal device
 */
void getDeviceMemoryInfo(double& availableMB, double& usedMB, double& totalMB, uint64_t deviceID);

}  // namespace depthMap
}  // namespace aliceVision
