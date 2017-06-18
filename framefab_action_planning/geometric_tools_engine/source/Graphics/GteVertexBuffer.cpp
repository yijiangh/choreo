// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/11/28)

#include <GTEnginePCH.h>
#include <Graphics/GteVertexBuffer.h>
#include <Graphics/GteStructuredBuffer.h>
using namespace gte;

VertexBuffer::VertexBuffer(VertexFormat const& vformat,
    unsigned int numVertices, bool createStorage)
    :
    Buffer(numVertices, vformat.GetVertexSize(), createStorage),
    mVFormat(vformat)
{
    mType = GT_VERTEX_BUFFER;
}

VertexBuffer::VertexBuffer(VertexFormat const& vformat,
    std::shared_ptr<StructuredBuffer> const& sbuffer)
    :
    Buffer(sbuffer->GetNumElements(), vformat.GetVertexSize(), false),
    mVFormat(vformat),
    mSBuffer(sbuffer)
{
    mType = GT_VERTEX_BUFFER;
    SetNumActiveElements(sbuffer->GetNumElements());
}

VertexBuffer::VertexBuffer(unsigned int numVertices)
    :
    Buffer(numVertices, 0, false)
{
    mType = GT_VERTEX_BUFFER;
}

char* VertexBuffer::GetChannel(VASemantic semantic, unsigned int unit,
    std::set<DFType> const& requiredTypes)
{
    char* data = (mSBuffer == nullptr ? mData : mSBuffer->GetData());
    if (!data)
    {
        // The system memory copy does not exist.  You need to recreate it
        // before populating it.
        return nullptr;
    }

    int index = mVFormat.GetIndex(semantic, unit);
    if (index < 0)
    {
        // The buffer does not have the specified semantic that uses the
        // specified unit.
        return nullptr;
    }

    DFType type = mVFormat.GetType(index);
    if (requiredTypes.size() > 0)
    {
        if (requiredTypes.find(type) == requiredTypes.end())
        {
            // The type of the semantic is not in the set of required types.
            return nullptr;
        }
    }

    return data + mVFormat.GetOffset(index);
}
