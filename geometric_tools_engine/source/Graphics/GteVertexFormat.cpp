// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GteVertexFormat.h>
using namespace gte;


VertexFormat::VertexFormat()
    :
    mNumAttributes(0),
    mVertexSize(0)
{
}

bool VertexFormat::Bind(VASemantic semantic, DFType type, unsigned int unit)
{
    if (0 <= mNumAttributes && mNumAttributes < VA_MAX_ATTRIBUTES)
    {
        // Validate the inputs.
        if (semantic == VA_COLOR)
        {
            if (unit >= VA_MAX_COLOR_UNITS)
            {
                LogError("Invalid color unit.");
                return false;
            }
        }
        else if (semantic == VA_TEXCOORD)
        {
            if (unit >= VA_MAX_TCOORD_UNITS)
            {
                LogError("Invalid texture unit.");
                return false;
            }
        }
        else
        {
            if (unit > 0)
            {
                LogError("Invalid semantic unit.");
                return false;
            }
        }

        // Accept the attribute.
        Attribute& attribute = mAttributes[mNumAttributes];
        attribute.semantic = semantic;
        attribute.type = type;
        attribute.unit = unit;
        attribute.offset = mVertexSize;
        ++mNumAttributes;

        // Advance the offset.
        mVertexSize += DataFormat::GetNumBytesPerStruct(type);
        return true;
    }

    LogError("Exceeded maximum attributes.");
    return false;
}

unsigned int VertexFormat::GetVertexSize() const
{
    return mVertexSize;
}

int VertexFormat::GetNumAttributes() const
{
    return mNumAttributes;
}

bool VertexFormat::GetAttribute(int i, VASemantic& semantic, DFType& type,
    unsigned int& unit, unsigned int& offset) const
{
    if (0 <= i && i < mNumAttributes)
    {
        Attribute const& attribute = mAttributes[i];
        semantic = attribute.semantic;
        type = attribute.type;
        unit = attribute.unit;
        offset = attribute.offset;
        return true;
    }

    LogError("Invalid index " + std::to_string(i) + ".");
    return false;
}

int VertexFormat::GetIndex(VASemantic semantic, unsigned int unit) const
{
    for (int i = 0; i < mNumAttributes; ++i)
    {
        Attribute const& attribute = mAttributes[i];
        if (attribute.semantic == semantic && attribute.unit == unit)
        {
            return i;
        }
    }

    return -1;
}

DFType VertexFormat::GetType(int i) const
{
    if (0 <= i && i < mNumAttributes)
    {
        return mAttributes[i].type;
    }

    LogError("Invalid index " + std::to_string(i) + ".");
    return DF_UNKNOWN;
}

unsigned int VertexFormat::GetOffset(int i) const
{
    if (0 <= i && i < mNumAttributes)
    {
        return mAttributes[i].offset;
    }

    LogError("Invalid index " + std::to_string(i) + ".");
    return 0xFFFFFFFFu;
}

VertexFormat::Attribute::Attribute()
    :
    semantic(VA_NO_SEMANTIC),
    type(DF_UNKNOWN),
    unit(0),
    offset(0)
{
}

