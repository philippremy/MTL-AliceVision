//
//  DownscaleWithGaussianBlur_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef DownscaleWithGaussianBlur_PushConstants_h
#define DownscaleWithGaussianBlur_PushConstants_h

namespace aliceVision {
namespace depthMap {

struct DownscaleWithGaussianBlur_PushConstants
{
    int downscale;
    int gaussRadius;
};

}
}

#endif /* DownscaleWithGaussianBlur_PushConstants_h */
