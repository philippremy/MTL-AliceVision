//
//  DepthThicknessSmoothThickness.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/DepthThicknessSmoothThickness_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void DepthThicknessSmoothThickness(constant DepthThicknessSmoothThickness_PushConstants& pushConstants [[buffer(0)]],
                                          texture2d<float, access::read_write> inout_depthThicknessMap_d [[texture(0)]],
                                          uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;
    
    // corresponding output depth/thickness (depth unchanged)
    const float4 inout_depthThickness = inout_depthThicknessMap_d.read(uint2(roiX, roiY));

    // depth invalid or masked
    if(inout_depthThickness.x <= 0.0f)
        return;

    const float minThickness = pushConstants.minThicknessInflate * inout_depthThickness.y;
    const float maxThickness = pushConstants.maxThicknessInflate * inout_depthThickness.y;
    
    // compute average depth distance to the center pixel
    float sumCenterDepthDist = 0.f;
    int nbValidPatchPixels = 0;
    
    // patch 3x3
    #pragma unroll
    for(int yp = -1; yp <= 1; ++yp)
    {
        #pragma unroll
        for(int xp = -1; xp <= 1; ++xp)
        {
            // compute patch coordinates
            const int roiXp = int(roiX) + xp;
            const int roiYp = int(roiY) + yp;

            if((xp == 0 && yp == 0) ||                // avoid pixel center
               roiXp < 0 || roiXp >= int(pushConstants.roi.width()) ||   // avoid pixel outside the ROI
               roiYp < 0 || roiYp >= int(pushConstants.roi.height()))    // avoid pixel outside the ROI
            {
                continue;
            }

            // corresponding path depth/thickness
            const float4 in_depthThicknessPatch = inout_depthThicknessMap_d.read(uint2(roiXp, roiYp));

            // patch depth valid
            if(in_depthThicknessPatch.x > 0.0f)
            {
                const float depthDistance = abs(inout_depthThickness.x - in_depthThicknessPatch.x);
                sumCenterDepthDist += max(minThickness, min(maxThickness, depthDistance)); // clamp (minThickness, maxThickness)
                ++nbValidPatchPixels;
            }
        }
    }
    
    // we require at least 3 valid patch pixels (over 8)
    if(nbValidPatchPixels < 3)
        return;
    
    // write output smooth thickness
    inout_depthThicknessMap_d.write(float4(inout_depthThickness.x, (sumCenterDepthDist / nbValidPatchPixels), inout_depthThickness.zw), uint2(roiX, roiY));
    
}

}
}