//
//  MapUpscale_PushConstants.hpp
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef MapUpscale_PushConstants_h
#define MapUpscale_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct MapUpscale_PushConstants
{
    float ratio;
    ROI roi;
};

}
}

#endif /* MapUpscale_PushConstants_h */
