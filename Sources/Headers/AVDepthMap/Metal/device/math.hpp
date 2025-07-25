// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

inline float norm3df(float a, float b, float c)
{
    if(isinf(a) || isinf(b) || isinf(c))
        return +INFINITY;
    if(isnan(a) || isnan(b)|| isnan(c))
        return NAN;
    if(a == 0.f && b == 0.f && c == 0.f)
        return +0.f;
    return sqrt((a * a) + (b * b) + (c * c));
}

inline float cbrtf(float x)
{
    constexpr float onethird = 1.f / 3.f;
    if (isnan(x))
        return NAN;
    if (isinf(x))
        return x;
    if (x == 0.0f)
        return 0.0f;
    return pow(x, onethird);
}

inline float fsqrt_rn(float x)
{
    if (isnan(x))
        return NAN;
    if (x < 0.0f)
        return NAN;
    if(x == 0.0f)
        return x;
    return sqrt(x);
}

inline float expf(float x)
{
    if(isnan(x))
        return NAN;
    if(x == -INFINITY)
        return +0;
    if(x == +INFINITY)
        return x;
    if(x == 0)
        return 1;
    return exp(x);
}
