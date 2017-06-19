// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteSamplerState.h>
using namespace gte;


SamplerState::SamplerState()
    :
    filter(MIN_P_MAG_P_MIP_P),
    mipLODBias(0.0f),
    maxAnisotropy(1),
    comparison(NEVER),
    borderColor({ 1.0f, 1.0f, 1.0f, 1.0f }),
    minLOD(-std::numeric_limits<float>::max()),
    maxLOD(std::numeric_limits<float>::max())
{
    mType = GT_SAMPLER_STATE;

    mode[0] = CLAMP;
    mode[1] = CLAMP;
    mode[2] = CLAMP;
}

