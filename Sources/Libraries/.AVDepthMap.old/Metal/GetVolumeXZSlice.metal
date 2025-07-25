//
//  GetVolumeXZSlice.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/GetVolumeXZSlice_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void GetVolumeXZSlice(constant GetVolumeXZSlice_PushConstants& pushConstants [[buffer(0)]],
                             texture2d<TSimAcc, access::read_write> slice_d [[texture(0)]],
                             texture3d<TSim, access::read> volume_d [[texture(1)]],
                             uint3 gid [[thread_position_in_grid]])
{
    
    const int x = gid.x;
    const int z = gid.y;

    uint3 v;
    v[pushConstants.axisT.x] = x;
    v[pushConstants.axisT.y] = pushConstants.y;
    v[pushConstants.axisT.z] = z;

    if (x >= pushConstants.volDim[pushConstants.axisT.x] || z >= pushConstants.volDim[pushConstants.axisT.z])
      return;

    TSim volume_xyz = volume_d.read(v).x;
    vec<TSimAcc, 4> slice_xz = slice_d.read(uint2(x, z));
    slice_xz.x = volume_xyz;
    slice_d.write(slice_xz, uint2(x, z));
    
}

}
}