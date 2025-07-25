// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

template<typename T>
VulkanCommandManager* VulkanCommandManager::transferImageLayout(const VulkanImage<T>& in_img, const vk::ImageLayout& targetLayout, const uint32_t mipLevelStart, const uint32_t mipLevelEnd)
{
    this->standardOp([&](const vk::SharedCommandBuffer& cmdBuffer){
        const vk::ImageSubresourceRange imageSubresourceRange( vk::ImageAspectFlagBits::eColor, mipLevelStart, mipLevelEnd - mipLevelStart, 0, 1 );
        const vk::ImageMemoryBarrier imageMemoryBarrier({vk::AccessFlagBits::eMemoryRead | vk::AccessFlagBits::eMemoryWrite}, {vk::AccessFlagBits::eMemoryRead | vk::AccessFlagBits::eMemoryWrite}, vk::ImageLayout::eUndefined, targetLayout, {}, {}, *in_img.getVkImage().value(), imageSubresourceRange);
        cmdBuffer->pipelineBarrier({vk::PipelineStageFlagBits::eAllCommands}, {vk::PipelineStageFlagBits::eAllCommands}, {}, {}, {}, { imageMemoryBarrier });
        in_img.m_layout = targetLayout;
    });
    return this;
}

template<typename T>
VulkanCommandManager* VulkanCommandManager::bind(const VulkanMemoryBase<T>& buffer, const vk::DescriptorType& descriptorType)
{
    this->m_boundMemoryInstances.emplace_back(std::make_unique<BoundMemory<T>>(buffer, buffer.getByteSize(), this->m_boundMemoryInstances.size(), descriptorType));
    return this;
}

template <typename Type>
VulkanCommandManager* VulkanCommandManager::bind(const VulkanMemoryBase<Type>& buffer, const vk::DescriptorType& descriptorType, uint32_t mipLevel)
{
    if(buffer.isVkBuffer() || descriptorType != vk::DescriptorType::eStorageImage) {
        throw std::runtime_error("This function is only valid for images and for descriptor type vk::DescriptorType::eStorageImage!");
    }
    return this;
}
