//
// Created by Philipp Remy on 18.07.25.
//

#ifndef DEVICEGAUSSIANARRAY_HPP
#define DEVICEGAUSSIANARRAY_HPP

#include <AVGPU/Metal/device.hpp>

namespace aliceVision {
namespace depthMap {

using namespace gpu;

#define MAX_CONSTANT_GAUSS_SCALES 10
#define MAX_CONSTANT_GAUSS_MEM_SIZE 128

class DeviceGaussianArray
{
public:
    explicit DeviceGaussianArray(uint64_t deviceID, uint64_t scales);
    const MTLSharedResource<MTLBuffer, int>& getGaussianArrayOffsetBuffer() const;
    const MTLSharedResource<MTLBuffer, float>& getGaussianArrayBuffer() const;

private:
    std::unique_ptr<MTLSharedResource<MTLBuffer, int>> m_gaussianArrayOffsetBuffer = nullptr;
    std::unique_ptr<MTLSharedResource<MTLBuffer, float>> m_gaussianArrayBuffer = nullptr;
};

}
}

#endif //DEVICEGAUSSIANARRAY_HPP
