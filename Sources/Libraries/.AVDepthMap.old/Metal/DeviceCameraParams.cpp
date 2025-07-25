//
// Created by Philipp Remy on 18.07.25.
//

#include <AVDepthMap/Metal/DeviceCameraParams.hpp>

namespace aliceVision {
namespace depthMap {

using namespace gpu;

DeviceCameraParamsArray::DeviceCameraParamsArray(uint64_t deviceID)
{
    const auto resMng = MTLDeviceManager::getInstance()->getResourceManager(deviceID);
    this->m_deviceCameraParamsArrayBuffer = std::make_unique<MTLSharedResource<MTLBuffer, DeviceCameraParams>>(resMng->createBuffer<DeviceCameraParams>(ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS, false));
}

void DeviceCameraParamsArray::addCameraParams(const DeviceCameraParams& cameraParams, uint64_t offset)
{
    this->m_deviceCameraParamsArrayBuffer->copyFromHost(&cameraParams, 1, 0, offset);
}

const MTLSharedResource<MTLBuffer, DeviceCameraParams>& DeviceCameraParamsArray::getDeviceCameraParamsBuffer() const
{
    return *this->m_deviceCameraParamsArrayBuffer;
}

}
}
