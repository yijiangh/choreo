// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11Texture1Array.h>
using namespace gte;

DX11Texture1Array::DX11Texture1Array(ID3D11Device* device, Texture1Array const* textureArray)
    :
    DX11TextureArray(textureArray)
{
    // Specify the texture description.
    D3D11_TEXTURE1D_DESC desc;
    desc.Width = textureArray->GetLength();
    desc.MipLevels = textureArray->GetNumLevels();
    desc.ArraySize = textureArray->GetNumItems();
    desc.Format = static_cast<DXGI_FORMAT>(textureArray->GetFormat());
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;
    Resource::Usage usage = textureArray->GetUsage();
    if (usage == Resource::IMMUTABLE)
    {
        desc.Usage = D3D11_USAGE_IMMUTABLE;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
    }
    else if (usage == Resource::DYNAMIC_UPDATE)
    {
        desc.Usage = D3D11_USAGE_DYNAMIC;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    }
    else  // usage == Resource::SHADER_OUTPUT
    {
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
    }

    if (textureArray->WantAutogenerateMipmaps())
    {
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
        desc.MiscFlags |= D3D11_RESOURCE_MISC_GENERATE_MIPS;
    }

    // Create the texture.
    ID3D11Texture1D* dxTexture = nullptr;
    HRESULT hr;
    if (textureArray->GetData())
    {
        unsigned int const numSubresources = textureArray->GetNumSubresources();
        std::vector<D3D11_SUBRESOURCE_DATA> data(numSubresources);
        for (unsigned int index = 0; index < numSubresources; ++index)
        {
            auto sr = textureArray->GetSubresource(index);
            data[index].pSysMem = sr.data;
            data[index].SysMemPitch = 0;
            data[index].SysMemSlicePitch = 0;
        }
        hr = device->CreateTexture1D(&desc, &data[0], &dxTexture);
    }
    else
    {
        hr = device->CreateTexture1D(&desc, nullptr, &dxTexture);
    }
    CHECK_HR_RETURN_NONE("Failed to create texture array");
    mDXObject = dxTexture;

    // Create views of the texture.
    CreateSRView(device, desc);
    if (usage == Resource::SHADER_OUTPUT)
    {
        CreateUAView(device, desc);
    }

    // Create a staging texture if requested.
    if (textureArray->GetCopyType() != Resource::COPY_NONE)
    {
        CreateStaging(device, desc);
    }

    // Generate mipmaps if requested.
    if (textureArray->WantAutogenerateMipmaps() && mSRView)
    {
        ID3D11DeviceContext* context;
        device->GetImmediateContext(&context);
        context->GenerateMips(mSRView);
        context->Release();
    }
}

std::shared_ptr<GEObject> DX11Texture1Array::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE1_ARRAY)
    {
        return std::make_shared<DX11Texture1Array>(reinterpret_cast<ID3D11Device*>(device),
            static_cast<Texture1Array const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void DX11Texture1Array::CreateStaging(ID3D11Device* device, D3D11_TEXTURE1D_DESC const& tx)
{
    D3D11_TEXTURE1D_DESC desc;
    desc.Width = tx.Width;
    desc.MipLevels = tx.MipLevels;
    desc.ArraySize = tx.ArraySize;
    desc.Format = tx.Format;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = msStagingAccess[GetTextureArray()->GetCopyType()];
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;

    HRESULT hr = device->CreateTexture1D(&desc, nullptr,
        reinterpret_cast<ID3D11Texture1D**>(&mStaging));
    CHECK_HR_RETURN_NONE("Failed to create staging texture");
}

void DX11Texture1Array::CreateSRView(ID3D11Device* device, D3D11_TEXTURE1D_DESC const& tx)
{
    D3D11_SHADER_RESOURCE_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1DARRAY;
    desc.Texture1DArray.MostDetailedMip = 0;
    desc.Texture1DArray.MipLevels = tx.MipLevels;
    desc.Texture1DArray.FirstArraySlice = 0;
    desc.Texture1DArray.ArraySize = tx.ArraySize;

    HRESULT hr = device->CreateShaderResourceView(GetDXTextureArray(), &desc, &mSRView);
    CHECK_HR_RETURN_NONE("Failed to create shader resource view");
}

void DX11Texture1Array::CreateUAView(ID3D11Device* device, D3D11_TEXTURE1D_DESC const& tx)
{
    D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_UAV_DIMENSION_TEXTURE1DARRAY;
    desc.Texture1DArray.MipSlice = 0;
    desc.Texture1DArray.FirstArraySlice = 0;
    desc.Texture1DArray.ArraySize = tx.ArraySize;

    HRESULT hr = device->CreateUnorderedAccessView(GetDXTextureArray(), &desc, &mUAView);
    CHECK_HR_RETURN_NONE("Failed to create unordered access view");
}
