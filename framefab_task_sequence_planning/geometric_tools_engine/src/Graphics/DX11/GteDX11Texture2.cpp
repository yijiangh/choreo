// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDX11Texture2.h>
using namespace gte;

DX11Texture2::DX11Texture2(ID3D11Device* device, Texture2 const* texture)
    :
    DX11TextureSingle(texture)
{
    // Specify the texture description.
    D3D11_TEXTURE2D_DESC desc;
    desc.Width = texture->GetWidth();
    desc.Height = texture->GetHeight();
    desc.MipLevels = texture->GetNumLevels();
    desc.ArraySize = 1;
    desc.Format = static_cast<DXGI_FORMAT>(texture->GetFormat());
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = (texture->IsShared() ?
        D3D11_RESOURCE_MISC_SHARED : D3D11_RESOURCE_MISC_NONE);
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

    if (texture->WantAutogenerateMipmaps() && !texture->IsShared())
    {
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_NONE;
        desc.MiscFlags |= D3D11_RESOURCE_MISC_GENERATE_MIPS;
    }

    // Create the texture.
    ID3D11Texture2D* dxTexture = nullptr;
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
            data[index].SysMemSlicePitch = 0;
        }
        hr = device->CreateTexture2D(&desc, &data[0], &dxTexture);
    }
    else
    {
        hr = device->CreateTexture2D(&desc, nullptr, &dxTexture);
    }
    CHECK_HR_RETURN_VOID("Failed to create texture");
    mDXObject = dxTexture;

    // Create views of the texture.
    CreateSRView(device, desc);
    if (texture->GetUsage() == Resource::SHADER_OUTPUT)
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

DX11Texture2::DX11Texture2(ID3D11Device* device, DX11Texture2 const* dxSharedTexture)
    :
    DX11TextureSingle(dxSharedTexture->GetTexture())
{
    ID3D11Texture2D* dxShared = dxSharedTexture->CreateSharedDXObject(device);
    mDXObject = dxShared;

    // Create view of the shared texture.
    auto gtTexture = dxSharedTexture->GetTexture();
    D3D11_TEXTURE2D_DESC desc;
    dxShared->GetDesc(&desc);
    CreateSRView(device, desc);
    if (gtTexture->GetUsage() == Resource::SHADER_OUTPUT)
    {
        CreateUAView(device, desc);
    }

    // Create a staging texture for the shared texture if the original texture
    // also has ones.
    if (gtTexture->GetCopyType() != Resource::COPY_NONE)
    {
        CreateStaging(device, desc);
    }
}

DX11Texture2::DX11Texture2(Texture2 const* texture, ID3D11Texture2D* dxTexture,
    ID3D11ShaderResourceView* dxSRView)
    :
    DX11TextureSingle(texture)
{
    mDXObject = dxTexture;
    mSRView = dxSRView;
}

std::shared_ptr<GEObject> DX11Texture2::Create(void* device, GraphicsObject const* object)
{
    if (object->GetType() == GT_TEXTURE2)
    {
        return std::make_shared<DX11Texture2>(
            reinterpret_cast<ID3D11Device*>(device),
            static_cast<Texture2 const*>(object));
    }

    LogError("Invalid object type.");
    return nullptr;
}

DX11Texture2::DX11Texture2(Texture2 const* texture)
    :
    DX11TextureSingle(texture)
{
}

ID3D11Texture2D* DX11Texture2::CreateSharedDXObject(ID3D11Device* device) const
{
    IDXGIResource* resource = nullptr;
    HRESULT hr = mDXObject->QueryInterface(__uuidof(IDXGIResource), (void**)&resource);
    CHECK_HR_RETURN("QueryInterface failed", nullptr);

    HANDLE handle = nullptr;
    hr = resource->GetSharedHandle(&handle);
    resource->Release();
    CHECK_HR_RETURN("GetSharedHandle failed", nullptr);

    ID3D11Texture2D* dxShared = nullptr;
    hr = device->OpenSharedResource(handle, __uuidof(ID3D11Texture2D), (void**)&dxShared);
    CHECK_HR_RETURN("OpenSharedResource failed", nullptr);
    return dxShared;
}

void DX11Texture2::CreateStaging(ID3D11Device* device, D3D11_TEXTURE2D_DESC const& tx)
{
    D3D11_TEXTURE2D_DESC desc;
    desc.Width = tx.Width;
    desc.Height = tx.Height;
    desc.MipLevels = tx.MipLevels;
    desc.ArraySize = tx.ArraySize;
    desc.Format = tx.Format;
    desc.SampleDesc.Count = tx.SampleDesc.Count;
    desc.SampleDesc.Quality = tx.SampleDesc.Quality;
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = D3D11_BIND_NONE;
    desc.CPUAccessFlags = msStagingAccess[GetTexture()->GetCopyType()];
    desc.MiscFlags = D3D11_RESOURCE_MISC_NONE;

    HRESULT hr = device->CreateTexture2D(&desc, nullptr, reinterpret_cast<ID3D11Texture2D**>(&mStaging));
    CHECK_HR_RETURN_NONE("Failed to create staging texture");
}

void DX11Texture2::CreateSRView(ID3D11Device* device, D3D11_TEXTURE2D_DESC const& tx)
{
    D3D11_SHADER_RESOURCE_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    desc.Texture2D.MostDetailedMip = 0;
    desc.Texture2D.MipLevels = tx.MipLevels;

    HRESULT hr = device->CreateShaderResourceView(GetDXTexture(), &desc, &mSRView);
    CHECK_HR_RETURN_NONE("Failed to create shader resource view");
}

void DX11Texture2::CreateUAView(ID3D11Device* device, D3D11_TEXTURE2D_DESC const& tx)
{
    D3D11_UNORDERED_ACCESS_VIEW_DESC desc;
    desc.Format = tx.Format;
    desc.ViewDimension = D3D11_UAV_DIMENSION_TEXTURE2D;
    desc.Texture2D.MipSlice = 0;

    HRESULT hr = device->CreateUnorderedAccessView(GetDXTexture(), &desc, &mUAView);
    CHECK_HR_RETURN_NONE("Failed to create unordered access view");
}
