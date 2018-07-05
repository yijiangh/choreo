// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteIndirectArgumentsBuffer.h>
using namespace gte;


IndirectArgumentsBuffer::IndirectArgumentsBuffer(unsigned int numElements,
    bool createStorage)
    :
    Buffer(numElements, 4, createStorage)
{
    mType = GT_INDIRECT_ARGUMENTS_BUFFER;
}

