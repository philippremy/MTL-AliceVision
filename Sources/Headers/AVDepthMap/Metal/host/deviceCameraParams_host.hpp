// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "DeviceManager.hpp"

#include <simd/vector.h>
using namespace simd;

#include <Metal/Metal.hpp>

#include <AVDepthMap/Metal/device/DeviceCameraParams.hpp>

namespace aliceVision {
namespace depthMap {

class DeviceCameraParamsManager
{
public:
  DeviceCameraParamsManager() = default;
  explicit DeviceCameraParamsManager(uint64_t deviceID)
  {
      const auto device = DeviceManager::getInstance().getDevice(deviceID);
      _camParamArray = NS::TransferPtr(device->newBuffer(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS * sizeof(DeviceCameraParams), MTL::ResourceStorageModePrivate));
      _stagingBuffer = NS::TransferPtr(device->newBuffer(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS * sizeof(DeviceCameraParams), MTL::ResourceStorageModeManaged));
  }

  ~DeviceCameraParamsManager() = default;

  void insertCameraParams(const DeviceCameraParams& camParams, int offset)
  {
        // Copy into staging buffer
        memcpy(_stagingBuffer->contents(), &camParams, sizeof(DeviceCameraParams));
        _stagingBuffer->didModifyRange(NS::Range(0, sizeof(DeviceCameraParams)));
        // Copy the data into the first mip level
        const auto queue = DeviceManager::getInstance().getCommandQueue(_camParamArray->device()->registryID());
        const auto commandBuffer = queue->commandBuffer();
        const auto blitCommandEncoder = commandBuffer->blitCommandEncoder();
        blitCommandEncoder->copyFromBuffer(_stagingBuffer.get(), 0, _camParamArray.get(), offset * sizeof(DeviceCameraParams), sizeof(DeviceCameraParams));
        blitCommandEncoder->endEncoding();
        commandBuffer->commit();
        commandBuffer->waitUntilCompleted();
  }

  inline MTL::Buffer* getCameraParamsBuffer() const { return _camParamArray.get(); }

private:
  NS::SharedPtr<MTL::Buffer> _camParamArray;
  NS::SharedPtr<MTL::Buffer> _stagingBuffer;
};

}
}
