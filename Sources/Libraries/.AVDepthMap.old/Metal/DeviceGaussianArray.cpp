//
// Created by Philipp Remy on 18.07.25.
//

#include <AVDepthMap/Metal/DeviceGaussianArray.hpp>

namespace aliceVision {
namespace depthMap {

DeviceGaussianArray::DeviceGaussianArray(const uint64_t deviceID, const uint64_t scales)
{
    // Fill the buffers
    if(scales >= MAX_CONSTANT_GAUSS_SCALES)
    {
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    int gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES];
    float gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE];

    int sumSizes = 0;

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        gaussianArrayOffset[scale] = sumSizes;
        const int radius = scale + 1;
        const int size = 2 * radius + 1;
        sumSizes += size;
    }

    if(sumSizes >= MAX_CONSTANT_GAUSS_MEM_SIZE)
    {
        throw std::runtime_error( "Programming error: too little memory allocated for "
            + std::to_string(MAX_CONSTANT_GAUSS_SCALES) + " Gaussian kernels. Enlarge and recompile." );
    }

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        const int radius = scale + 1;
        const float delta  = 1.0f;
        const int size   = 2 * radius + 1;

        for(int idx = 0; idx < size; idx++)
        {
            int x = idx - radius;
            gaussianArray[gaussianArrayOffset[scale]+idx] = expf(-(x * x) / (2 * delta * delta));
        }
    }

    // Create buffers
    const auto resMng = MTLDeviceManager::getInstance()->getResourceManager(deviceID);
    m_gaussianArrayOffsetBuffer = std::make_unique<MTLSharedResource<MTLBuffer, int>>(resMng->createBuffer<int>(MAX_CONSTANT_GAUSS_SCALES, false));
    m_gaussianArrayBuffer = std::make_unique<MTLSharedResource<MTLBuffer, float>>(resMng->createBuffer<float>(MAX_CONSTANT_GAUSS_MEM_SIZE, false));

    // Copy into buffers
    m_gaussianArrayOffsetBuffer->copyFromHost(gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES, 0, 0);
    m_gaussianArrayBuffer->copyFromHost(gaussianArray, MAX_CONSTANT_GAUSS_MEM_SIZE, 0, 0);
}

const MTLSharedResource<MTLBuffer, float>& DeviceGaussianArray::getGaussianArrayBuffer() const
{
    return *this->m_gaussianArrayBuffer;
}

const MTLSharedResource<MTLBuffer, int>& DeviceGaussianArray::getGaussianArrayOffsetBuffer() const
{
    return *this->m_gaussianArrayOffsetBuffer;
}

}
}