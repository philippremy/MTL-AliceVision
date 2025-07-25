// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVDepthMap/ComputeOnMultiGPUs.hpp>

#include <vector>

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

/**
 * @brief Normal Map Estimator
 * @brief Wrap normal maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class NormalMapEstimator : public IGPUJob
{
  public:
    /**
     * @brief Normal Map Estimator constructor.
     * @param[in] mp the multi-view parameters
     */
    NormalMapEstimator(const mvsUtils::MultiViewParams& mp);

    // no copy constructor
    NormalMapEstimator(NormalMapEstimator const&) = delete;

    // no copy operator
    void operator=(NormalMapEstimator const&) = delete;

    // destructor
    ~NormalMapEstimator() = default;

    /**
     * @brief Compute normal maps of the given cameras.
     * @param[in] cudaDeviceId the CUDA device id
     * @param[in] cams the list of cameras
     */
    void compute(int cudaDeviceId, const std::vector<int>& cams) override;

  private:
    // private members

    const mvsUtils::MultiViewParams& _mp;  //< multi-view parameters
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

/**
 * @brief Normal Map Estimator
 * @brief Wrap normal maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class NormalMapEstimator : public IGPUJob
{
  public:
    /**
     * @brief Normal Map Estimator constructor.
     * @param[in] mp the multi-view parameters
     */
    NormalMapEstimator(const mvsUtils::MultiViewParams& mp);

    // no copy constructor
    NormalMapEstimator(NormalMapEstimator const&) = delete;

    // no copy operator
    void operator=(NormalMapEstimator const&) = delete;

    // destructor
    ~NormalMapEstimator() = default;

    /**
     * @brief Compute normal maps of the given cameras.
     * @param[in] cudaDeviceId the CUDA device id
     * @param[in] cams the list of cameras
     */
    void compute(uint32_t deviceID, const std::vector<int>& cams) override;

  private:
    // private members

    const mvsUtils::MultiViewParams& _mp;  //< multi-view parameters
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

/**
 * @brief Normal Map Estimator
 * @brief Wrap normal maps estimation computation.
 * @note Allows muli-GPUs computation (interface IGPUJob)
 */
class NormalMapEstimator : public IGPUJob
{
public:
    /**
     * @brief Normal Map Estimator constructor.
     * @param[in] mp the multi-view parameters
     */
    NormalMapEstimator(const mvsUtils::MultiViewParams& mp);

    // no copy constructor
    NormalMapEstimator(NormalMapEstimator const&) = delete;

    // no copy operator
    void operator=(NormalMapEstimator const&) = delete;

    // destructor
    ~NormalMapEstimator() = default;

    /**
     * @brief Compute normal maps of the given cameras.
     * @param[in] mtlDeviceID the Metal device id
     * @param[in] cams the list of cameras
     */
    void compute(uint64_t mtlDeviceID, const std::vector<int>& cams) override;

private:
    // private members

    const mvsUtils::MultiViewParams& _mp;  //< multi-view parameters
};

#else
  #error No backend for DepthMap computations was selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision
