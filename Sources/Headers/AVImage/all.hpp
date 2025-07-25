// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// Get rid of the specific MSVC compiler warnings.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
    #define _CRT_SECURE_NO_WARNINGS
#endif

#include <AVImage/Image.hpp>
#include <AVImage/pixelTypes.hpp>
#include <AVImage/conversion.hpp>
#include <AVImage/drawing.hpp>
#include <AVImage/filtering.hpp>
#include <AVImage/resampling.hpp>
#include <AVImage/diffusion.hpp>
#include <AVImage/concat.hpp>
#include <AVImage/imageAlgo.hpp>
#include <AVImage/io.hpp>
#include <AVImage/convolutionBase.hpp>
#include <AVImage/convolution.hpp>
#include <AVImage/Rgb.hpp>
#include <AVImage/Sampler.hpp>
#include <AVImage/conversionOpenCV.hpp>
#include <AVImage/dcp.hpp>
