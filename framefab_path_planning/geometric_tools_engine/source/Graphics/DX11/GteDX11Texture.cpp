// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteWrapper.h>
#include <Graphics/DX11/GteDX11Texture.h>
using namespace gte;

DX11Texture::~DX11Texture()
{
    FinalRelease(mSRView);
    FinalRelease(mUAView);
}

DX11Texture::DX11Texture(Texture const* gtTexture)
    :
    DX11Resource(gtTexture),
    mSRView(nullptr),
    mUAView(nullptr)
{
}

bool DX11Texture::Update(ID3D11DeviceContext* context, unsigned int sri)
{
    Texture* texture = GetTexture();
    if (sri >= texture->GetNumSubresources())
    {
        LogWarning("Subresource index out of range.");
        return false;
    }

    if (texture->GetUsage() != Resource::DYNAMIC_UPDATE)
    {
        LogWarning("Texture usage is not DYNAMIC_UPDATE.");
        return false;
    }

    // Map the texture.
    ID3D11Resource* dxTexture = GetDXResource();
    D3D11_MAPPED_SUBRESOURCE sub;
    HRESULT hr = context->Map(dxTexture, sri, D3D11_MAP_WRITE_DISCARD, 0, &sub);
    CHECK_HR_RETURN("Failed to map staging texture", false);

    // Copy from CPU memory.
    auto sr = texture->GetSubresource(sri);
    unsigned int numDimensions = texture->GetNumDimensions();
    if (numDimensions == 1)
    {
        Memcpy(sub.pData, sr.data, texture->GetNumBytesFor(sr.level));
    }
    else if (numDimensions == 2)
    {
        CopyPitched2(texture->GetDimensionFor(sr.level, 1), sr.rowPitch,
            sr.data, sub.RowPitch, sub.pData);
    }
    else  // numDimensions == 3
    {
        CopyPitched3(texture->GetDimensionFor(sr.level, 1),
            texture->GetDimensionFor(sr.level, 2), sr.rowPitch, sr.slicePitch,
            sr.data, sub.RowPitch, sub.DepthPitch, sub.pData);
    }
    context->Unmap(dxTexture, sri);
    return true;
}

bool DX11Texture::Update(ID3D11DeviceContext* context)
{
    Texture* texture = GetTexture();
    unsigned int const numSubresources = texture->GetNumSubresources();
    for (unsigned int index = 0; index < numSubresources; ++index)
    {
        if (!Update(context, index))
        {
            return false;
        }
    }
    return true;
}

bool DX11Texture::CopyCpuToGpu(ID3D11DeviceContext* context,
    unsigned int sri)
{
    Texture* texture = GetTexture();
    if (sri >= texture->GetNumSubresources())
    {
        LogWarning("Subresource index out of range.");
        return false;
    }

    if (!PreparedForCopy(D3D11_CPU_ACCESS_WRITE))
    {
        return false;
    }

    // Map the staging texture.
    D3D11_MAPPED_SUBRESOURCE sub;
    HRESULT hr = context->Map(mStaging, sri, D3D11_MAP_WRITE, 0, &sub);
    CHECK_HR_RETURN("Failed to map staging texture", false);

    // Copy from CPU memory to staging texture.
    auto sr = texture->GetSubresource(sri);
    unsigned int numDimensions = texture->GetNumDimensions();
    if (numDimensions == 1)
    {
        Memcpy(sub.pData, sr.data, texture->GetNumBytesFor(sr.level));
    }
    else if (numDimensions == 2)
    {
        CopyPitched2(texture->GetDimensionFor(sr.level, 1), sr.rowPitch,
            sr.data, sub.RowPitch, sub.pData);
    }
    else  // numDimensions == 3
    {
        CopyPitched3(texture->GetDimensionFor(sr.level, 1),
            texture->GetDimensionFor(sr.level, 2), sr.rowPitch,
            sr.slicePitch, sr.data, sub.RowPitch, sub.DepthPitch,
            sub.pData);
    }
    context->Unmap(mStaging, sri);

    // Copy from staging texture to GPU memory.
    ID3D11Resource* dxTexture = GetDXResource();
    context->CopySubresourceRegion(dxTexture, sri, 0, 0, 0, mStaging, sri, nullptr);
    return true;
}

bool DX11Texture::CopyCpuToGpu(ID3D11DeviceContext* context)
{
    Texture* texture = GetTexture();
    unsigned int const numSubresources = texture->GetNumSubresources();
    for (unsigned int index = 0; index < numSubresources; ++index)
    {
        if (!CopyCpuToGpu(context, index))
        {
            return false;
        }
    }

    // Generate mipmaps (when they exist).
    if (texture->WantAutogenerateMipmaps() && mSRView)
    {
        context->GenerateMips(mSRView);
    }
    return true;
}

bool DX11Texture::CopyGpuToCpu(ID3D11DeviceContext* context, unsigned int sri)
{
    Texture* texture = GetTexture();
    if (sri >= texture->GetNumSubresources())
    {
        LogWarning("Subresource index out of range.");
        return false;
    }

    if (!PreparedForCopy(D3D11_CPU_ACCESS_READ))
    {
        return false;
    }

    // Copy from GPU memory to staging texture.
    ID3D11Resource* dxTexture = GetDXResource();
    context->CopySubresourceRegion(mStaging, sri, 0, 0, 0, dxTexture, sri, nullptr);

    // Map the staging texture.
    D3D11_MAPPED_SUBRESOURCE sub;
    HRESULT hr = context->Map(mStaging, sri, D3D11_MAP_READ, 0, &sub);
    CHECK_HR_RETURN("Failed to map staging texture", false);

    // Copy from staging texture to CPU memory.
    auto sr = texture->GetSubresource(sri);
    unsigned int numDimensions = texture->GetNumDimensions();
    if (numDimensions == 1)
    {
        Memcpy(sr.data, sub.pData, texture->GetNumBytesFor(sr.level));
    }
    else if (numDimensions == 2)
    {
        CopyPitched2(texture->GetDimensionFor(sr.level, 1), sub.RowPitch,
            sub.pData, sr.rowPitch, sr.data);
    }
    else  // numDimensions == 3
    {
        CopyPitched3(texture->GetDimensionFor(sr.level, 1),
            texture->GetDimensionFor(sr.level, 2), sub.RowPitch,
            sub.DepthPitch, sub.pData, sr.rowPitch, sr.slicePitch, sr.data);
    }
    context->Unmap(mStaging, sri);
    return true;
}

bool DX11Texture::CopyGpuToCpu(ID3D11DeviceContext* context)
{
    Texture* texture = GetTexture();
    unsigned int const numSubresources = texture->GetNumSubresources();
    for (unsigned int index = 0; index < numSubresources; ++index)
    {
        if (!CopyGpuToCpu(context, index))
        {
            return false;
        }
    }
    return true;
}

void DX11Texture::CopyGpuToGpu(ID3D11DeviceContext* context,
    ID3D11Resource* target, unsigned int sri)
{
    Texture* texture = GetTexture();
    if (sri >= texture->GetNumSubresources())
    {
        LogWarning("Subresource index out of range.");
        return;
    }

    // Copy from GPU memory to staging texture.
    ID3D11Resource* dxTexture = GetDXResource();
    context->CopySubresourceRegion(target, sri, 0, 0, 0, dxTexture, sri, nullptr);
}

void DX11Texture::CopyGpuToGpu(ID3D11DeviceContext* context, ID3D11Resource* target)
{
    Texture* texture = GetTexture();
    unsigned int const numSubresources = texture->GetNumSubresources();
    for (unsigned int index = 0; index < numSubresources; ++index)
    {
        CopyGpuToGpu(context, target, index);
    }
}

void DX11Texture::SetName(std::string const& name)
{
    DX11Resource::SetName(name);
    HRESULT hr = SetPrivateName(mSRView, name);
    CHECK_HR_RETURN_NONE("Failed to set private name");
    hr = SetPrivateName(mUAView, name);
    CHECK_HR_RETURN_NONE("Failed to set private name");
}

void DX11Texture::CopyPitched2(unsigned int numRows,
    unsigned int srcRowPitch, void const* srcData, unsigned int trgRowPitch,
    void* trgData)
{
    if (srcRowPitch == trgRowPitch)
    {
        // The memory is contiguous.
        Memcpy(trgData, srcData, trgRowPitch*numRows);
    }
    else
    {
        // Padding was added to each row of the texture, so we must
        // copy a row at a time to compensate for differing pitches.
        unsigned int numRowBytes = std::min(srcRowPitch, trgRowPitch);
        char const* srcRow = static_cast<char const*>(srcData);
        char* trgRow = static_cast<char*>(trgData);
        for (unsigned int row = 0; row < numRows; ++row)
        {
            Memcpy(trgRow, srcRow, numRowBytes);
            srcRow += srcRowPitch;
            trgRow += trgRowPitch;
        }
    }
}

void DX11Texture::CopyPitched3(unsigned int numRows,
    unsigned int numSlices, unsigned int srcRowPitch,
    unsigned int srcSlicePitch, void const* srcData, unsigned int trgRowPitch,
    unsigned int trgSlicePitch, void* trgData)
{
    if (srcRowPitch == trgRowPitch && srcSlicePitch == trgSlicePitch)
    {
        // The memory is contiguous.
        Memcpy(trgData, srcData, trgSlicePitch*numSlices);
    }
    else
    {
        // Padding was added to each row and/or slice of the texture, so
        // we must copy the data to compensate for differing pitches.
        unsigned int numRowBytes = std::min(srcRowPitch, trgRowPitch);
        char const* srcSlice = static_cast<char const*>(srcData);
        char* trgSlice = static_cast<char*>(trgData);
        for (unsigned int slice = 0; slice < numSlices; ++slice)
        {
            char const* srcRow = srcSlice;
            char* trgRow = trgSlice;
            for (unsigned int row = 0; row < numRows; ++row)
            {
                Memcpy(trgRow, srcRow, numRowBytes);
                srcRow += srcRowPitch;
                trgRow += trgRowPitch;
            }
            srcSlice += srcSlicePitch;
            trgSlice += trgSlicePitch;
        }
    }
}
