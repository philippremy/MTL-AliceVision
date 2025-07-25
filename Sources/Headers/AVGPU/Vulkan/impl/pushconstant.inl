// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

template<typename PushConstants>
VulkanCommandManager* VulkanCommandManager::pushConstant(const PushConstants& pushConstants)
{
    this->m_pushConstantTemporaryStorage.first = static_cast<const void*>(&pushConstants);
    this->m_pushConstantTemporaryStorage.second = sizeof(PushConstants);
    return this;
}
