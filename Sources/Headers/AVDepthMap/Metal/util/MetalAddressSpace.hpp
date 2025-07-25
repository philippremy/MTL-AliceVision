// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

#if defined(__METAL__)
#define METAL_QUALIFIER_DEVICE device
#define METAL_QUALIFIER_CONSTANT constant
#else
#define METAL_QUALIFIER_DEVICE
#define METAL_QUALIFIER_CONSTANT
#endif

enum class MetalAddressSpace
{
    Constant,
    Device,
};

template<MetalAddressSpace A> struct MetalAddressSpaceTag {};

}
}