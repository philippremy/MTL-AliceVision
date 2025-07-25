//
//  VolumeRefineSimiliarity_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef VolumeRefineSimiliarity_PushConstants_h
#define VolumeRefineSimiliarity_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct VolumeRefineSimiliarity_PushConstants
{
    bool hasNormalMap;
    int rcDeviceCameraParamsId;
    int tcDeviceCameraParamsId;
    unsigned int rcRefineLevelWidth;
    unsigned int rcRefineLevelHeight;
    unsigned int tcRefineLevelWidth;
    unsigned int tcRefineLevelHeight;
    float rcMipmapLevel;
    int volDimZ;
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

#endif /* VolumeRefineSimiliarity_PushConstants_h */
