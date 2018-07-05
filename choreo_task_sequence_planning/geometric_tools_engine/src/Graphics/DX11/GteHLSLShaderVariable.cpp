// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/09/12)

#include <GTEnginePCH.h>
#include <LowLevel/GteWrapper.h>
#include <Graphics/DX11/GteHLSLShaderVariable.h>
#include <iomanip>
using namespace gte;


HLSLShaderVariable::HLSLShaderVariable()
{
}

void HLSLShaderVariable::SetDescription(
    D3D_SHADER_VARIABLE_DESC const& desc)
{
    mDesc.name = std::string(desc.Name ? desc.Name : "");
    mDesc.offset = desc.StartOffset;
    mDesc.numBytes = desc.Size;
    mDesc.flags = desc.uFlags;
    mDesc.textureStart = desc.StartTexture;
    mDesc.textureNumSlots = desc.TextureSize;
    mDesc.samplerStart = desc.StartSampler;
    mDesc.samplerNumSlots = desc.SamplerSize;
    if (desc.DefaultValue && desc.Size > 0)
    {
        mDesc.defaultValue.resize(desc.Size);
        Memcpy(&mDesc.defaultValue[0], desc.DefaultValue, desc.Size);
    }
}

std::string const& HLSLShaderVariable::GetName() const
{
    return mDesc.name;
}

unsigned int HLSLShaderVariable::GetOffset() const
{
    return mDesc.offset;
}

unsigned int HLSLShaderVariable::GetNumBytes() const
{
    return mDesc.numBytes;
}

unsigned int HLSLShaderVariable::GetFlags() const
{
    return mDesc.flags;
}

unsigned int HLSLShaderVariable::GetTextureStart() const
{
    return mDesc.textureStart;
}

unsigned int HLSLShaderVariable::GetTextureNumSlots() const
{
    return mDesc.textureNumSlots;
}

unsigned int HLSLShaderVariable::GetSamplerStart() const
{
    return mDesc.samplerStart;
}

unsigned int HLSLShaderVariable::GetSamplerNumSlots() const
{
    return mDesc.samplerNumSlots;
}

std::vector<unsigned char> const& HLSLShaderVariable::GetDefaultValue() const
{
    return mDesc.defaultValue;
}

void HLSLShaderVariable::Print(std::ofstream& output) const
{
    output << "name = " << mDesc.name << std::endl;
    output << "offset = " << mDesc.offset << std::endl;
    output << "numBytes = " << mDesc.numBytes << std::endl;
    output << "flags = " << mDesc.flags << std::endl;

    if (mDesc.textureStart == 0xFFFFFFFF)
    {
        output << "textureStart = -1" << std::endl;
    }
    else
    {
        output << "textureStart = " << mDesc.textureStart << std::endl;
    }
    output << "textureNumSlots = " << mDesc.textureNumSlots << std::endl;

    if (mDesc.samplerStart == 0xFFFFFFFF)
    {
        output << "samplerStart = -1" << std::endl;
    }
    else
    {
        output << "samplerStart = " << mDesc.samplerStart << std::endl;
    }
    output << "textureNumSlots = " << mDesc.samplerNumSlots << std::endl;

    output << "default value = ";
    size_t size = mDesc.defaultValue.size();
    if (size > 0)
    {
        output << std::hex << std::endl;
        size_t j = 0;
        for (auto c : mDesc.defaultValue)
        {
            unsigned int hc = static_cast<unsigned int>(c);
            output << "0x" << std::setw(2) << std::setfill('0') << hc;
            if ((++j % 16) == 0)
            {
                if (j != size)
                {
                    output << std::endl;
                }
            }
            else
            {
                output << ' ';
            }
        }
        output << std::dec;
    }
    else
    {
        output << "none";
    }
    output << std::endl;
}

