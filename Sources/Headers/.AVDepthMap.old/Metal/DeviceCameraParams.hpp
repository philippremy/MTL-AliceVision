//
// Created by Philipp Remy on 17.07.25.
//

#ifndef DEVICECAMERAPARAMS_HPP
#define DEVICECAMERAPARAMS_HPP

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#else
#include <simd/vector.h>
using namespace simd;
#include <AVGPU/Metal/device.hpp>
#endif

namespace aliceVision {
namespace depthMap {

#define ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS 100

/**
 * @struct DeviceCameraParams
 * @brief Support class to maintain useful camera parameters in gpu memory.
 */
struct DeviceCameraParams
{
    float P[12];
    float iP[9];
    float R[9];
    float iR[9];
    float K[9];
    float iK[9];
    float3 C;
    float3 XVect;
    float3 YVect;
    float3 ZVect;
};

#if !defined(__METAL__)

using namespace gpu;

class DeviceCameraParamsArray
{
public:
    explicit DeviceCameraParamsArray(uint64_t deviceID);
    void addCameraParams(const DeviceCameraParams& cameraParams, uint64_t offset);
    const MTLSharedResource<MTLBuffer, DeviceCameraParams>& getDeviceCameraParamsBuffer() const;

private:
    std::unique_ptr<MTLSharedResource<MTLBuffer, DeviceCameraParams>> m_deviceCameraParamsArrayBuffer = nullptr;
};

#endif

}
}


#endif //DEVICECAMERAPARAMS_HPP
