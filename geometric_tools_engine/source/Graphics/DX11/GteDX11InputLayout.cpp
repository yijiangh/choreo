// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11InputLayout.h>
using namespace gte;

DX11InputLayout::~DX11InputLayout()
{
    FinalRelease(mLayout);
}

DX11InputLayout::DX11InputLayout(ID3D11Device* device,
    VertexBuffer const* vbuffer, Shader const* vshader)
    :
    mLayout(nullptr),
    mNumElements(0)
{
    memset(&mElements[0], 0, VA_MAX_ATTRIBUTES*sizeof(mElements[0]));
    if (vbuffer && vshader)
    {
        VertexFormat const& format = vbuffer->GetFormat();
        mNumElements = format.GetNumAttributes();
        for (int i = 0; i < mNumElements; ++i)
        {
            VASemantic semantic;
            DFType type;
            unsigned int unit, offset;
            format.GetAttribute(i, semantic, type, unit, offset);

            D3D11_INPUT_ELEMENT_DESC& element = mElements[i];
            element.SemanticName = msSemantic[semantic];
            element.SemanticIndex = unit;
            element.Format = static_cast<DXGI_FORMAT>(type);
            element.InputSlot = 0;  // TODO: Streams not yet supported.
            element.AlignedByteOffset = offset;
            element.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
            element.InstanceDataStepRate = 0;
        }

        std::vector<unsigned char> const& compiledCode =
            vshader->GetCompiledCode();
        HRESULT hr = device->CreateInputLayout(mElements, (UINT)mNumElements,
            &compiledCode[0], compiledCode.size(), &mLayout);
        CHECK_HR_RETURN_NONE("Failed to create input layout");
    }
    else
    {
        LogError("Invalid inputs to DX11InputLayout constructor.");
    }
}

void DX11InputLayout::Enable(ID3D11DeviceContext* context)
{
    if (mLayout)
    {
        context->IASetInputLayout(mLayout);
    }
}

void DX11InputLayout::Disable(ID3D11DeviceContext* context)
{
    if (mLayout)
    {
        // TODO: Verify that mLayout is the active input layout.
        context->IASetInputLayout(nullptr);
    }
}

HRESULT DX11InputLayout::SetName(std::string const& name)
{
    mName = name;
    return SetPrivateName(mLayout, mName);
}


char const* DX11InputLayout::msSemantic[VA_NUM_SEMANTICS] =
{
    "",
    "POSITION",
    "BLENDWEIGHT",
    "BLENDINDICES",
    "NORMAL",
    "PSIZE",
    "TEXCOORD",
    "TANGENT",
    "BINORMAL",
    "TESSFACTOR",
    "POSITIONT",
    "COLOR",
    "FOG",
    "DEPTH",
    "SAMPLE"
};
