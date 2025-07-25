//
// Created by Philipp Remy on 17.07.25.
//

#ifndef DEVICEPATCHPATTERN_HPP
#define DEVICEPATCHPATTERN_HPP

#if defined(__METAL__)
#include <metal_stdlib>
using namespace metal;
#else
#include <simd/vector.h>
using namespace simd;
#include <AVGPU/Metal/device.hpp>
#include <AVDepthMap/CustomPatchPatternParams.hpp>
#endif

namespace aliceVision {
namespace depthMap {

// maximum number of patch pattern subparts
// note: each patch pattern subpart gives one similarity
#define ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS 4

// maximum number of coordinates per patch pattern subpart
#define ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS 24

/**
 * @struct DevicePatchPatternSubpart
 *
 * @brief Support class to maintain a subpart of a patch pattern in gpu constant memory.
 *        Each patch pattern subpart gives one similarity score.
 *
 * @note Should be entirely initialize from host memory.
 *       CUDA doesn't support default initialization for struct in constant memory.
 */
struct DevicePatchPatternSubpart
{
    float2 coordinates[ALICEVISION_DEVICE_PATCH_MAX_COORDS_PER_SUBPARTS];  //< subpart coordinate list
    int nbCoordinates;                                                     //< subpart number of coordinate
    float level;                                                           //< subpart related mipmap level (>=0)
    float downscale;                                                       //< subpart related mipmap downscale (>=1)
    float weight;                                                          //< subpart related similarity weight in range (0, 1)
    bool isCircle;                                                         //< subpart is a circle
    int wsh;                                                               //< subpart half-width (full and circle)
};

/**
 * @struct DevicePatchPattern
 * @brief Support class to maintain a patch pattern in gpu constant memory.
 */
struct DevicePatchPattern
{
    DevicePatchPatternSubpart subparts[ALICEVISION_DEVICE_PATCH_MAX_SUBPARTS];  //< patch pattern subparts (one similarity per subpart)
    int nbSubparts;                                                             //< patch pattern number of subparts (>0)
};

#if !defined(__METAL__)

using namespace gpu;

class DevicePatchPatternArray
{
public:
    explicit DevicePatchPatternArray(uint64_t deviceID);
    void fillDevicePatchPatternBuffer(const CustomPatchPatternParams& patchParams);
    const MTLSharedResource<MTLBuffer, DevicePatchPattern>& getDevicePatchPatternBuffer() const;

private:
    std::unique_ptr<MTLSharedResource<MTLBuffer, DevicePatchPattern>> m_devicePatchPatternBuffer = nullptr;
};

#endif

}
}

#endif //DEVICEPATCHPATTERN_HPP
