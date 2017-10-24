// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/06/26)

#include <GTEnginePCH.h>
#include <Graphics/GtePickRecord.h>
using namespace gte;

PickRecord::~PickRecord()
{
}

PickRecord::PickRecord()
    :
    primitiveType(IP_NONE),
    primitiveIndex(0),
    t(0.0f),
    linePoint({ 0.0f, 0.0f, 0.0f, 1.0f }),
    primitivePoint({ 0.0f, 0.0f, 0.0f, 1.0f }),
    distanceToLinePoint(0.0f),
    distanceToPrimitivePoint(0.0f),
    distanceBetweenLinePrimitive(0.0f)
{
    for (int i = 0; i < 3; ++i)
    {
        vertexIndex[i] = 0;
        bary[i] = 0.0f;
    }
}
