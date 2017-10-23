// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteStructuredBuffer.h>
using namespace gte;


StructuredBuffer::StructuredBuffer(unsigned int numElements,
    size_t elementSize, bool createStorage)
    :
    Buffer(numElements, elementSize, createStorage),
    mCounterType(CT_NONE),
    mKeepInternalCount(false)
{
    mType = GT_STRUCTURED_BUFFER;
}

StructuredBuffer::CounterType StructuredBuffer::GetCounterType() const
{
    return mCounterType;
}

void StructuredBuffer::MakeAppendConsume()
{
    mCounterType = CT_APPEND_CONSUME;
    mUsage = SHADER_OUTPUT;
}

void StructuredBuffer::MakeCounter()
{
    mCounterType = CT_COUNTER;
    mUsage = SHADER_OUTPUT;
}

bool StructuredBuffer::GetKeepInternalCount() const
{
    return mKeepInternalCount;
}

void StructuredBuffer::SetKeepInternalCount(bool keepInternalCount)
{
    if (mCounterType != CT_NONE)
    {
        mKeepInternalCount = keepInternalCount;
    }
}

