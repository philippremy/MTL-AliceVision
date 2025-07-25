//
//  DepthThicknessSmoothThickness_PushConstants.h
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#ifndef DepthThicknessSmoothThickness_PushConstants_h
#define DepthThicknessSmoothThickness_PushConstants_h

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

struct DepthThicknessSmoothThickness_PushConstants
{
    float minThicknessInflate;
    float maxThicknessInflate;
    ROI roi;
};

}
}

#endif /* DepthThicknessSmoothThickness_PushConstants_h */
