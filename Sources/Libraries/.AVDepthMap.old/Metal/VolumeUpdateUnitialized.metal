//
//  VolumeUpdateUnitialized.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void VolumeUpdateUnitialized(texture3d<TSim, access::read_write> inout_volume2nd_d [[texture(0)]],
                                    texture3d<TSim, access::read> in_volume1st_d [[texture(1)]],
                                    uint3 gid [[thread_position_in_grid]],
                                    uint3 bit [[threadgroup_position_in_grid]])
{
        
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;
    const unsigned int vz = bit.z;

    if(vx >= inout_volume2nd_d.get_width() || vy >= inout_volume2nd_d.get_height())
        return;

    // input/output second best similarity value
    vec<TSim, 4> inout_sim = inout_volume2nd_d.read(uint3(vx, vy, vz));

    if(inout_sim.x >= 255.f) // invalid or uninitialized similarity value
    {
        // update second best similarity value with first best similarity value
        inout_sim.x = in_volume1st_d.read(uint3(vx, vy, vz)).x;
        inout_volume2nd_d.write(inout_sim, uint3(vx, vy, vz));
    }
    
}

}
}