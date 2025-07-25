//
// Created by Philipp Remy on 17.07.25.
//

#pragma once

#include <AVGPU/Metal/device.hpp>

namespace aliceVision {
namespace gpu {

template<ResourceType Resource, typename T>
MTLCommandManager* MTLCommandManager::bind(const MTLSharedResource<Resource, T>& resource, const uint64_t idx)
{
    if constexpr (std::is_same_v<Resource, MTLBuffer>)  // Binding a buffer
    {
        this->m_commandEncoder->setBuffer(resource.getResource().get(), 0, idx);
    }
    else  // Binding a Texture
    {
        this->m_commandEncoder->setTexture(resource.getResource().get(), idx);
    }
    return this;
}

template<typename T>
MTLCommandManager* MTLCommandManager::pushConstants(const T& pushConstants)
{
    static_assert(sizeof(T) < 4096, "Direct bindings in Metal must be smaller than 4KB. The provided PushConstants are bigger than 4KB!");
    this->m_commandEncoder->setBytes(static_cast<const void*>(&pushConstants), sizeof(T), 0);
    return this;
}


}  // namespace gpu
}  // namespace aliceVision