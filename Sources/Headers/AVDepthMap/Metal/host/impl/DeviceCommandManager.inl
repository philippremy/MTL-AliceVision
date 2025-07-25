//
// Created by Philipp Remy on 17.07.25.
//

#pragma once

namespace aliceVision {
namespace depthMap {

template<typename T>
MTLCommandManager* MTLCommandManager::pushConstants(const T& pushConstants)
{
    static_assert(sizeof(T) < 4096, "Direct bindings in Metal must be smaller than 4KB. The provided PushConstants are bigger than 4KB!");
    this->m_commandEncoder->setBytes(static_cast<const void*>(&pushConstants), sizeof(T), 30);
    return this;
}

template<typename T>
MTLCommandManager* MTLCommandManager::constant(const T& constant, uint64_t idx)
{
    static_assert(sizeof(T) < 4096, "Direct bindings in Metal must be smaller than 4KB. The provided PushConstants are bigger than 4KB!");
    this->m_commandEncoder->setBytes(static_cast<const void*>(&constant), sizeof(T), idx);
    return this;
}

}  // namespace gpu
}  // namespace aliceVision