//
//  InitVolumeYSlice.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/InitVolumeYSlice_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void InitVolumeYSlice(constant InitVolumeYSlice_PushConstants& pushConstants [[buffer(0)]],
                             texture3d<TSim, access::read_write> volume_d [[texture(0)]],
                             uint3 gid [[thread_position_in_grid]])
{
    
    const int x = gid.x;
    const int z = gid.y;

    uint3 temp;
    temp[pushConstants.axisT.x] = x;
    temp[pushConstants.axisT.y] = pushConstants.y;
    temp[pushConstants.axisT.z] = z;

    if((x >= 0) && (x < pushConstants.volDim[pushConstants.axisT.x]) && (z >= 0) && (z < pushConstants.volDim[pushConstants.axisT.z])) {
        vec<TSim, 4> volume_zyx = volume_d.read(temp);
        volume_zyx.x = pushConstants.cst;
        volume_d.write(volume_zyx, temp);
    }
    
}

}
}