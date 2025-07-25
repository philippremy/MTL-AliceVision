// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/CustomPatchPatternParams.hpp>
#include <AVDepthMap/Metal/device/DevicePatchPattern.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Build user custom patch pattern in CUDA constant memory.
 * @param[in] patchParams the user custom patch pattern parameters
 */
void buildCustomPatchPattern(const CustomPatchPatternParams& patchParams, uint64_t deviceID);

class CustomPatchPattern
{
public:
  void addPatchPattern(const DevicePatchPattern& pattern, uint64_t deviceID);
  std::optional<const DevicePatchPattern> getDevicePatchPattern(uint64_t deviceID) const;

  static CustomPatchPattern& getInstance();

  CustomPatchPattern(const CustomPatchPattern& rhs) = delete;
  CustomPatchPattern operator=(const CustomPatchPattern& rhs) = delete;

private:
  CustomPatchPattern() = default;
  static CustomPatchPattern* _instance;
  std::unordered_map<uint64_t, DevicePatchPattern> _devicePatchPatterns;
};

}  // namespace depthMap
}  // namespace aliceVision
