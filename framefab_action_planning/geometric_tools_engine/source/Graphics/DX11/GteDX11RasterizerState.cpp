// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11RasterizerState.h>
using namespace gte;

DX11RasterizerState::DX11RasterizerState(ID3D11Device* device, RasterizerState const* rasterizerState)
    :
    DX11DrawingState(rasterizerState)
{
    // Specify the rasterizer state description.
    D3D11_RASTERIZER_DESC desc;
    desc.FillMode = msFillMode[rasterizerState->fillMode];
    desc.CullMode = msCullMode[rasterizerState->cullMode];
    desc.FrontCounterClockwise = (rasterizerState->frontCCW ? TRUE : FALSE);
    desc.DepthBias = rasterizerState->depthBias;
    desc.DepthBiasClamp = rasterizerState->depthBiasClamp;
    desc.SlopeScaledDepthBias = rasterizerState->slopeScaledDepthBias;
    desc.DepthClipEnable = (rasterizerState->enableDepthClip ? TRUE : FALSE);
    desc.ScissorEnable = (rasterizerState->enableScissor ? TRUE : FALSE);
    desc.MultisampleEnable = (rasterizerState->enableMultisample ? TRUE : FALSE);
    desc.AntialiasedLineEnable = (rasterizerState->enableAntialiasedLine ? TRUE : FALSE);

    // Create the rasterizer state.
    ID3D11RasterizerState* state = nullptr;
    HRESULT hr = device->CreateRasterizerState(&desc, &state);
    CHECK_HR_RETURN_NONE("Failed to create rasterizer state");
    mDXObject = state;
}

std::shared_ptr<GEObject> DX11RasterizerState::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_RASTERIZER_STATE)
    {
        return std::make_shared<DX11RasterizerState>(
            reinterpret_cast<ID3D11Device*>(device),
            static_cast<RasterizerState const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void DX11RasterizerState::Enable(ID3D11DeviceContext* context)
{
    context->RSSetState(GetDXRasterizerState());
}


D3D11_FILL_MODE const DX11RasterizerState::msFillMode[] =
{
    D3D11_FILL_SOLID,
    D3D11_FILL_WIREFRAME
};

D3D11_CULL_MODE const DX11RasterizerState::msCullMode[] =
{
    D3D11_CULL_NONE,
    D3D11_CULL_FRONT,
    D3D11_CULL_BACK
};
