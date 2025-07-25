// This file is part of the extension to AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVMVSUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @class IGPUJob
 * @brief Interface for multi-GPUs computation.
 */
class IGPUJob
{
  public:
    /**
     * @brief Perform computation from the given cameras.
     * @param[in] mtlDeviceID the CUDA device id
     * @param[in] cams the list of cameras
     */
    virtual void compute(uint64_t mtlDeviceID, const std::vector<int>& cams) = 0;
};

/**
 * @brief Perform computation from the given cameras on multiple GPUs.
 * @param[in] cams the given list of cameras
 * @param[in,out] gpujob the object that wrap computation (should use IGPUJob interface)
 * @param[in] nbGPUsToUse the number of GPUs to use
 */
void computeOnMultiGPUs(const std::vector<int>& cams, IGPUJob& gpujob, int nbGPUsToUse);

}  // namespace depthMap
}  // namespace aliceVision
