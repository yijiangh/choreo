// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11GeometryShader.h>
using namespace gte;

DX11GeometryShader::DX11GeometryShader(ID3D11Device* device, Shader const* shader)
    :
    DX11Shader(shader)
{
    std::vector<unsigned char> const& code = shader->GetCompiledCode();

    ID3D11ClassLinkage* linkage = nullptr;
    ID3D11GeometryShader* dxShader = nullptr;
    HRESULT hr = device->CreateGeometryShader(&code[0], code.size(), linkage, &dxShader);
    CHECK_HR_RETURN_NONE("Failed to create geometry shader");

    mDXObject = dxShader;
}

std::shared_ptr<GEObject> DX11GeometryShader::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_GEOMETRY_SHADER)
    {
        return std::make_shared<DX11GeometryShader>(
            reinterpret_cast<ID3D11Device*>(device),
            static_cast<Shader const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void DX11GeometryShader::Enable(ID3D11DeviceContext* context)
{
    if (mDXObject)
    {
        ID3D11ClassInstance* instances[1] = { nullptr };
        UINT numInstances = 0;
        ID3D11GeometryShader* dxShader = static_cast<ID3D11GeometryShader*>(mDXObject);
        context->GSSetShader(dxShader, instances, numInstances);
    }
}

void DX11GeometryShader::Disable(ID3D11DeviceContext* context)
{
    if (mDXObject)
    {
        ID3D11ClassInstance* instances[1] = { nullptr };
        UINT numInstances = 0;
        ID3D11GeometryShader* dxShader = nullptr;
        context->GSSetShader(dxShader, instances, numInstances);
    }
}

void DX11GeometryShader::EnableCBuffer(ID3D11DeviceContext* context,
    unsigned int bindPoint, ID3D11Buffer* buffer)
{
    if (mDXObject)
    {
        ID3D11Buffer* buffers[1] = { buffer };
        context->GSSetConstantBuffers(bindPoint, 1, buffers);
    }
}

void DX11GeometryShader::DisableCBuffer(ID3D11DeviceContext* context,
    unsigned int bindPoint)
{
    if (mDXObject)
    {
        ID3D11Buffer* buffers[1] = { nullptr };
        context->GSSetConstantBuffers(bindPoint, 1, buffers);
    }
}

void DX11GeometryShader::EnableSRView(ID3D11DeviceContext* context,
    unsigned int bindPoint, ID3D11ShaderResourceView* srView)
{
    if (mDXObject)
    {
        ID3D11ShaderResourceView* views[1] = { srView };
        context->GSSetShaderResources(bindPoint, 1, views);
    }
}

void DX11GeometryShader::DisableSRView(ID3D11DeviceContext* context,
    unsigned int bindPoint)
{
    if (mDXObject)
    {
        ID3D11ShaderResourceView* views[1] = { nullptr };
        context->GSSetShaderResources(bindPoint, 1, views);
    }
}

void DX11GeometryShader::EnableUAView(ID3D11DeviceContext* context,
    unsigned int bindPoint, ID3D11UnorderedAccessView* uaView,
    unsigned int initialCount)
{
    if (mDXObject)
    {
        ID3D11Device* device = nullptr;
        context->GetDevice(&device);
        if (!device)
        {
            LogError("Cannot access device of context.");
            return;
        }

        if (device->GetFeatureLevel() == D3D_FEATURE_LEVEL_11_1)
        {
            ID3D11UnorderedAccessView* uaViews[1] = { uaView };
            unsigned int initialCounts[1] = { initialCount };
            context->OMSetRenderTargetsAndUnorderedAccessViews(
                D3D11_KEEP_RENDER_TARGETS_AND_DEPTH_STENCIL, nullptr,
                nullptr, bindPoint, 1, uaViews, initialCounts);
        }
        else
        {
            LogError("D3D11.1 is required for UAVs in geometry shaders.");
        }
        device->Release();
    }
}

void DX11GeometryShader::DisableUAView(ID3D11DeviceContext* context,
    unsigned int bindPoint)
{
    if (mDXObject)
    {
        ID3D11Device* device = nullptr;
        context->GetDevice(&device);
        if (!device)
        {
            LogError("Cannot access device of context.");
            return;
        }

        if (device->GetFeatureLevel() == D3D_FEATURE_LEVEL_11_1)
        {
            ID3D11UnorderedAccessView* uaViews[1] = { nullptr };
            unsigned int initialCounts[1] = { 0xFFFFFFFFu };
            context->OMSetRenderTargetsAndUnorderedAccessViews(
                D3D11_KEEP_RENDER_TARGETS_AND_DEPTH_STENCIL, nullptr,
                nullptr, bindPoint, 1, uaViews, initialCounts);
        }
        else
        {
            LogError("D3D11.1 is required for UAVs in geometry shaders.");
        }
        device->Release();
    }
}

void DX11GeometryShader::EnableSampler(ID3D11DeviceContext* context,
    unsigned int bindPoint, ID3D11SamplerState* state)
{
    if (mDXObject)
    {
        ID3D11SamplerState* states[1] = { state };
        context->GSSetSamplers(bindPoint, 1, states);
    }
}

void DX11GeometryShader::DisableSampler(ID3D11DeviceContext* context,
    unsigned int bindPoint)
{
    if (mDXObject)
    {
        ID3D11SamplerState* states[1] = { nullptr };
        context->GSSetSamplers(bindPoint, 1, states);
    }
}
