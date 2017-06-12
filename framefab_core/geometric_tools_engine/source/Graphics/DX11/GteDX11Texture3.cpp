// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11Texture3.h>
using namespace gte;

DX11Texture3::DX11Texture3(ID3D11Device* device, Texture3 const* texture)
    :
    DX11TextureSingle(texture)
{
    // Specify the texture description.
    D3D11_TEXTURE3D_DESC desc;
    desc.Width = texture->GetWidth();
    desc.Height = texture->GetHeight();
    desc.Depth = texture->GetThickness();
    desc.MipLevels = texture->GetNumLevels();
    desc.Format = static_cast<DXGI_FORMAT>(texture->GetFormat());
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;
    Resource::Usage usage = texture->GetUsage();
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

    if (texture->WantAutogenerateMipmaps())
    {
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
        desc.MiscFlags |= D3D11_RESOURCE_MISC_GENERATE_MIPS;
    }

    // Create the texture.
    ID3D11Texture3D* dxTexture = nullptr;
    HRESULT hr;
    if (texture->GetData())
    {
        unsigned int const numSubresources = texture->GetNumSubresources();
        std::vector<D3D11_SUBRESOURCE_DATA> data(numSubresources);
        for (unsigned int index = 0; index < numSubresources; ++index)
        {
            auto sr = texture->GetSubresource(index);
            data[index].pSysMem = sr.data;
            data[index].SysMemPitch = sr.rowPitch;
            data[index].SysMemSlicePitch = sr.slicePitch;
        }
        hr = device->CreateTexture3D(&desc, &data[0], &dxTexture);
    }
    else
    {
        hr = device->CreateTexture3D(&desc, nullptr, &dxTexture);
    }
    CHECK_HR_RETURN_VOID("Failed to create texture");
    mDXObject = dxTexture;

    // Create views of the texture.
    CreateSRView(device, desc);
    if (usage == Resource::SHADER_OUTPUT)
    {
        CreateUAView(device, desc);
    }

    // Create a staging texture if requested.
    if (texture->GetCopyType() != Resource::COPY_NONE)
    {
        CreateStaging(device, desc);
    }

    // Generate mipmaps if requested.
    if (texture->WantAutogenerateMipmaps() && mSRView)
    {
        ID3D11DeviceContext* context;
        device->GetImmediateContext(&context);
        context->GenerateMips(mSRView);
        context->Release();
    }
}

std::shared_ptr<GEObject> DX11Texture3::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE3)
    {
        return std::make_shared<DX11Texture3>(reinterpret_cast<ID3D11Device*>(device),
            static_cast<Texture3 const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

void DX11Texture3::CreateStaging(ID3D11Device* device, D3D11_TEXTURE3D_DESC const& tx)
{
    D3D11_TEXTURE3D_DESC desc;
    desc.Width = tx.Width;
    desc.Height = tx.Height;
    desc.Depth = tx.Depth;
    desc.MipLevels = tx.MipLevels;
    desc.Format = tx.Format;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = msStagingAccess[GetTexture()->GetCopyType()];
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;

    HRESULT hr = device->CreateTexture3D(&desc, nullptr,
        reinterpret_cast<ID3D11Texture3D**>(&mStaging));
    CHECK_HR_RETURN_NONE("Failed to create staging texture");
}

void DX11Texture3::CreateSRView(ID3D11Device* device, D3D11_TEXTURE3D_DESC const& tx)
{
    D3D11_SHADER_RESOURCE_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE3D;
    desc.Texture3D.MostDetailedMip = 0;
    desc.Texture3D.MipLevels = tx.MipLevels;
    HRESULT hr = device->CreateShaderResourceView(GetDXTexture(), &desc, &mSRView);
    CHECK_HR_RETURN_NONE("Failed to create shader resource view");
}

void DX11Texture3::CreateUAView(ID3D11Device* device, D3D11_TEXTURE3D_DESC const& tx)
{
    D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_UAV_DIMENSION_TEXTURE3D;
    desc.Texture3D.MipSlice = 0;
    desc.Texture3D.FirstWSlice = 0;
    desc.Texture3D.WSize = tx.Depth;
    HRESULT hr = device->CreateUnorderedAccessView(GetDXTexture(), &desc, &mUAView);
    CHECK_HR_RETURN_NONE("Failed to create unordered access view");
}
