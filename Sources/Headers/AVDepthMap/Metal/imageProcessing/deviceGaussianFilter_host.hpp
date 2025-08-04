// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/host/memory.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/util/MTLMipmappedTexture.hpp>

namespace aliceVision {
namespace depthMap {

void mtl_downscaleWithGaussianBlur(MTLDeviceMemoryPitched<MTLRGBA, 2>& out_downscaledImg_dmp,
                                   const MTLMipmappedTexture<MTLRGBA>& in_img_tex,
                                   int downscale,
                                   int gaussRadius,
                                   uint64_t deviceID);

class DeviceGaussianFilterManager
{
public:
  DeviceGaussianFilterManager() = default;
  ~DeviceGaussianFilterManager() = default;

  void insertOffsetBuffer(const NS::SharedPtr<MTL::Buffer>& buffer) { _offsetBuffer = buffer; }
  void insertGaussianArrayBuffer(const NS::SharedPtr<MTL::Buffer>& buffer) { _gaussianArray = buffer; }

  inline MTL::Buffer* getOffsetBuffer() const { return _offsetBuffer.get(); }
  inline MTL::Buffer* getGaussianArrayBuffer() const { return _gaussianArray.get(); }

private:
  NS::SharedPtr<MTL::Buffer> _offsetBuffer;
  NS::SharedPtr<MTL::Buffer> _gaussianArray;
};

void mtl_createConstantGaussianArray(DeviceGaussianFilterManager& mng, uint64_t deviceID, int scales);

}
}
