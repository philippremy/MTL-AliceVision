// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#define MAX_CONSTANT_GAUSS_SCALES 10
#define MAX_CONSTANT_GAUSS_MEM_SIZE 128

#if defined(__METAL__)

inline float getGauss(constant int* d_gaussianArrayOffset, constant float* d_gaussianArray, int scale, int idx)
{
    return d_gaussianArray[d_gaussianArrayOffset[scale] + idx];
}

#endif

