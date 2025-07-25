// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <vulkan/vulkan_hpp_macros.hpp>
#include <vulkan/vulkan_core.h>

// Create the dispatcher in case of a dynamic loader
#if VULKAN_HPP_DISPATCH_LOADER_DYNAMIC == 1
    VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE
#endif

#define VMA_IMPLEMENTATION
#define VMA_VULKAN_VERSION 1002000 // Vulkan 1.2
#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 1
#include "vk_mem_alloc.h"
