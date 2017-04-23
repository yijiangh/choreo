// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/09/12)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteHLSLBaseBuffer.h>
using namespace gte;


HLSLBaseBuffer::~HLSLBaseBuffer()
{
}

HLSLBaseBuffer::HLSLBaseBuffer(D3D_SHADER_INPUT_BIND_DESC const& desc,
    unsigned int numBytes, std::vector<Member> const& members)
    :
    HLSLResource(desc, numBytes),
    mMembers(members)
{
}

HLSLBaseBuffer::HLSLBaseBuffer(D3D_SHADER_INPUT_BIND_DESC const& desc,
    unsigned int index, unsigned int numBytes,
    std::vector<Member> const& members)
    :
    HLSLResource(desc, index, numBytes),
    mMembers(members)
{
}

std::vector<HLSLBaseBuffer::Member> const& HLSLBaseBuffer::GetMembers() const
{
    return mMembers;
}

void HLSLBaseBuffer::Print(std::ofstream& output) const
{
    int i = 0;
    for (auto const& member : mMembers)
    {
        output << "Variable[" << i << "]:" << std::endl;
        member.first.Print(output);
        output << "Type[" << i << "]:" << std::endl;
        member.second.Print(output, 0);
        ++i;
    }
}

void HLSLBaseBuffer::GenerateLayout(std::vector<MemberLayout>& layout) const
{
    for (auto const& m : mMembers)
    {
        HLSLShaderType const& parent = m.second;
        GenerateLayout(parent, m.first.GetOffset(), parent.GetName(),
            layout);
    }
}

void HLSLBaseBuffer::GenerateLayout(HLSLShaderType const& parent,
    unsigned int parentOffset, std::string const& parentName,
    std::vector<MemberLayout>& layout) const
{
    unsigned int const numChildren = parent.GetNumChildren();
    if (numChildren > 0)
    {
        for (unsigned int i = 0; i < numChildren; ++i)
        {
            HLSLShaderType const& child = parent.GetChild(i);
            GenerateLayout(child, parentOffset + child.GetOffset(),
                parentName + "." + child.GetName(), layout);
        }
    }
    else
    {
        MemberLayout item;
        item.name = parentName;
        item.offset = parentOffset;
        item.numElements = parent.GetNumElements();
        layout.push_back(item);
    }
}

