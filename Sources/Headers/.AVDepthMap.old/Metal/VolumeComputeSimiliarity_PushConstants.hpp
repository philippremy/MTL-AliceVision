//
//  VolumeComputeSimiliarity_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef VolumeComputeSimiliarity_PushConstants_h
#define VolumeComputeSimiliarity_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct VolumeComputeSimiliarity_PushConstants
{
    int rcDeviceCameraParamsId;
    int tcDeviceCameraParamsId;
    unsigned int rcSgmLevelWidth;
    unsigned int rcSgmLevelHeight;
    unsigned int tcSgmLevelWidth;
    unsigned int tcSgmLevelHeight;
    float rcMipmapLevel;
    int stepXY;
    int wsh;
    float invGammaC;
    float invGammaP;
    bool useConsistentScale;
    bool useCustomPatchPattern;
    Range depthRange;
    ROI roi;
};

}
}

#endif /* VolumeComputeSimiliarity_PushConstants_h */
