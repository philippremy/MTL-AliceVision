// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/imageProcessing/deviceColorConversion_host.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

void mtl_rgb2lab(MTLDeviceMemoryPitched<MTLRGBA, 2>& inout_img_dmp, uint64_t deviceID)
{
    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const rgb2lab_kernel_PC pc = rgb2lab_kernel_PC{
        static_cast<unsigned int>(inout_img_dmp.getPitch()),
        static_cast<unsigned int>(inout_img_dmp.getSize().x()),
        static_cast<unsigned int>(inout_img_dmp.getSize().y())
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::rgb2lab_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_img_dmp.getBuffer(), 0)
    ->pushConstants(pc)
    ->dispatchDimensions({inout_img_dmp.getSize().x(), inout_img_dmp.getSize().y(), 1}, {32, 2, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

}
}