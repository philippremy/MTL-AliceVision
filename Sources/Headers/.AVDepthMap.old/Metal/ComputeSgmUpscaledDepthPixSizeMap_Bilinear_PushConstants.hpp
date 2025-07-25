//
//  ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants_h
#define ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants
{
    int rcDeviceCameraParamsId;
    unsigned int rcLevelWidth;
    unsigned int rcLevelHeight;
    float rcMipmapLevel;
    int stepXY;
    int halfNbDepths;
    float ratio;
    ROI roi;
};

}
}

#endif /* ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants_h */
