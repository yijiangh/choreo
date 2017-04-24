// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Applications/GteEnvironment.h>
#include <Applications/MSW/GteWICFileIO.h>
#include <memory>
// wincodec.h includes windows.h, so we must turn off min/max macros
#define NOMINMAX
#include <wincodec.h>
using namespace gte;

std::shared_ptr<Texture2> WICFileIO::Load(std::string const& filename, bool wantMipmaps)
{
    // Start COM and create WIC.
    ComInitializer comInitializer;
    if (!comInitializer.IsInitialized())
    {
        LogError("Unable to initialize COM for WIC.");
        return nullptr;
    }

    // Create a WIC imaging factory.
    ComObject<IWICImagingFactory> wicFactory;
    HRESULT hr = ::CoCreateInstance(CLSID_WICImagingFactory, nullptr,
        CLSCTX_INPROC_SERVER, IID_IWICImagingFactory,
        reinterpret_cast<LPVOID*>(&wicFactory));
    if (FAILED(hr))
    {
        LogError("Unable to create WIC imaging factory.");
        return nullptr;
    }

    // Create a decoder based on the file name.
    std::wstring wfilename(filename.begin(), filename.end());
    ComObject<IWICBitmapDecoder> wicDecoder;
    hr = wicFactory->CreateDecoderFromFilename(wfilename.c_str(),
        nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &wicDecoder);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateDecoderFromFilename failed (" +
            filename + ").");
        return nullptr;
    }

    // Create a WIC decoder.
    ComObject<IWICBitmapFrameDecode> wicFrameDecode;
    hr = wicDecoder->GetFrame(0, &wicFrameDecode);
    if (FAILED(hr))
    {
        LogError("wicDecoder->GetFrame failed.");
        return nullptr;
    }

    // Get the pixel format of the image.
    WICPixelFormatGUID wicSourceGUID;
    hr = wicFrameDecode->GetPixelFormat(&wicSourceGUID);
    if (FAILED(hr))
    {
        LogError("wicFrameDecode->GetPixelFormat failed.");
        return nullptr;
    }

    // Find the supported WIC input pixel format that matches a Texture2
    // format.  If a matching format is not found, the returned texture
    // is an R8G8B8A8 format with texels converted from the source format.
    WICPixelFormatGUID wicConvertGUID = GUID_WICPixelFormat32bppRGBA;
    DFType gtformat = DF_R8G8B8A8_UNORM;
    for (int i = 0; i < NUM_LOAD_FORMATS; ++i)
    {
        if (IsEqualGUID(wicSourceGUID, *msLoadFormatMap[i].wicInputGUID))
        {
            // Determine whether there is a conversion format.
            if (msLoadFormatMap[i].wicConvertGUID)
            {
                wicConvertGUID = *msLoadFormatMap[i].wicConvertGUID;
            }
            else
            {
                wicConvertGUID = *msLoadFormatMap[i].wicInputGUID;
            }
            gtformat = msLoadFormatMap[i].gtFormat;
            break;
        }
    }

    // The wicFrameDecode value is used for no conversion.  If the decoder
    // does not support the format in the texture, then a conversion is
    // required.
    IWICBitmapSource* wicBitmapSource = wicFrameDecode;
    ComObject<IWICFormatConverter> wicFormatConverter;
    if (!IsEqualGUID(wicSourceGUID, wicConvertGUID))
    {
        // Create a WIC format converter.
        hr = wicFactory->CreateFormatConverter(&wicFormatConverter);
        if (FAILED(hr))
        {
            LogError("wicFactory->CreateFormatConverter failed.");
            return nullptr;
        }

        // Initialize format converter to convert the input texture format
        // to the nearest format supported by the decoder.
        hr = wicFormatConverter->Initialize(wicFrameDecode, wicConvertGUID,
            WICBitmapDitherTypeNone, nullptr, 0.0,
            WICBitmapPaletteTypeCustom);
        if (FAILED(hr))
        {
            LogError("wicFormatConverter->Initialize failed.");
            return nullptr;
        }

        // Use the format converter.
        wicBitmapSource = wicFormatConverter;
    }

    // Get the image dimensions.
    UINT width, height;
    hr = wicBitmapSource->GetSize(&width, &height);
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->GetSize failed.");
        return nullptr;
    }

    // Create the 2D texture and compute the stride and image size.
    std::shared_ptr<Texture2> texture = std::make_shared<Texture2>(
        gtformat, width, height, wantMipmaps);
    UINT const stride = width * texture->GetElementSize();
    UINT const imageSize = stride * height;

    // Copy the pixels from the decoder to the texture.
    hr = wicBitmapSource->CopyPixels(nullptr, stride, imageSize,
        texture->Get<BYTE>());
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->CopyPixels failed.");
        return nullptr;
    }

    return texture;
}

bool WICFileIO::SaveToPNG(std::string const& filename,
    std::shared_ptr<Texture2> const& texture)
{
    return SaveTo(filename, texture, -1.0f);
}

bool WICFileIO::SaveToJPEG(std::string const& filename,
    std::shared_ptr<Texture2> const& texture, float imageQuality)
{
    imageQuality = std::min(std::max(imageQuality, 0.0f), 1.0f);
    return SaveTo(filename, texture, imageQuality);
}

bool WICFileIO::SaveTo(std::string const& filename,
    std::shared_ptr<Texture2> const& texture, float imageQuality)
{
    if (!texture || !texture->GetData())
    {
        LogError("The texture and its data must exist.");
        return false;
    }

    // Select the WIC format that matches the input texture format.
    WICPixelFormatGUID wicSourceGUID = GUID_WICPixelFormatUndefined;
    for (int i = 0; i < NUM_SAVE_FORMATS; ++i)
    {
        if (msSaveFormatMap[i].gtFormat == texture->GetFormat())
        {
            wicSourceGUID = *msSaveFormatMap[i].wicOutputGUID;
            break;
        }
    }
    if (IsEqualGUID(wicSourceGUID, GUID_WICPixelFormatUndefined))
    {
        LogError("Format " +
            DataFormat::GetName(texture->GetFormat()) +
            "is not supported for saving.");
        return false;
    }

    // Start COM and create WIC.
    ComInitializer comInitializer;
    if (!comInitializer.IsInitialized())
    {
        LogError("Unable to initialize COM for WIC.");
        return false;
    }

    // Create a WIC imaging factory.
    ComObject<IWICImagingFactory> wicFactory;
    HRESULT hr = ::CoCreateInstance(CLSID_WICImagingFactory, nullptr,
        CLSCTX_INPROC_SERVER, IID_IWICImagingFactory,
        reinterpret_cast<LPVOID*>(&wicFactory));
    if (FAILED(hr))
    {
        LogError("Unable to create WIC imaging factory.");
        return false;
    }

    // Create a WIC stream for output.
    ComObject<IWICStream> wicStream;
    hr = wicFactory->CreateStream(&wicStream);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateStream failed.");
        return false;
    }

    std::wstring wfilename(filename.begin(), filename.end());
    hr = wicStream->InitializeFromFilename(wfilename.c_str(), GENERIC_WRITE);
    if (FAILED(hr))
    {
        LogError("wicStream->InitializeFromFilename failed (" +
            filename + ").");
        return false;
    }

    // Create a WIC JPEG encoder.
    ComObject<IWICBitmapEncoder> wicEncoder;
    if (imageQuality == -1.0f)
    {
        hr = wicFactory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &wicEncoder);
    }
    else
    {
        hr = wicFactory->CreateEncoder(GUID_ContainerFormatJpeg, nullptr, &wicEncoder);
    }
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateEncoder failed.");
        return false;
    }

    hr = wicEncoder->Initialize(wicStream, WICBitmapEncoderNoCache);
    if (FAILED(hr))
    {
        LogError("wicEncoder->Initialize failed.");
        return false;
    }

    // Create a new frame and a property bag for encoder options.
    ComObject<IWICBitmapFrameEncode> wicFrameEncode;
    ComObject<IPropertyBag2> wicPropertyBag;
    hr = wicEncoder->CreateNewFrame(&wicFrameEncode, &wicPropertyBag);
    if (FAILED(hr))
    {
        LogError("wicEncoder->CreateNewFrame failed.");
        return false;
    }

    if (imageQuality == -1.0f)
    {
        // Set the options for the PNG encoder.
        PROPBAG2 option = { 0 };
        VARIANT varValue;

        // Default subsampling.
        option.pstrName = L"InterlaceOption";
        VariantInit(&varValue);
        varValue.vt = VT_BOOL;
        varValue.boolVal = FALSE;
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for InterlaceOption.");
            return false;
        }

        // Disable filtering.
        option.pstrName = L"FilterOption";
        VariantInit(&varValue);
        varValue.vt = VT_UI1;
        varValue.bVal = WICPngFilterNone;
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for FilterOption.");
            return false;
        }
    }
    else
    {
        // Set the options for the PNG encoder.
        PROPBAG2 option = { 0 };
        VARIANT varValue;

        // Set image quality, a number in [0,1].
        option.pstrName = L"ImageQuality";
        VariantInit(&varValue);
        varValue.vt = VT_R4;
        varValue.fltVal = imageQuality;
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for ImageQuality.");
            return false;
        }
    }

    // Initialize the encoder.
    hr = wicFrameEncode->Initialize(wicPropertyBag);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->Initialize failed.");
        return false;
    }

    // Set the image size.
    UINT width = texture->GetWidth();
    UINT height = texture->GetHeight();
    hr = wicFrameEncode->SetSize(width, height);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->SetSize failed.");
        return false;
    }

    // Set the image format.
    WICPixelFormatGUID wicTargetGUID = wicSourceGUID;
    hr = wicFrameEncode->SetPixelFormat(&wicTargetGUID);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->SetPixelFormat failed.");
        return false;
    }

    // Compute the stride and image size.
    UINT const stride = width * texture->GetElementSize();
    UINT const imageSize = stride * height;

    // Create a WIC bitmap to wrap the texture image data.
    ComObject<IWICBitmap> wicTextureBitmap;
    hr = wicFactory->CreateBitmapFromMemory(width, height,
        wicSourceGUID, stride, imageSize, texture->Get<BYTE>(),
        &wicTextureBitmap);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateBitmapFromMemory failed.");
        return false;
    }

    // The wicTextureBitmap value is used for no conversion.  If the encoder
    // does not support the format in the texture, then a conversion is
    // required.
    IWICBitmapSource* wicBitmapSource = wicTextureBitmap;
    ComObject<IWICFormatConverter> wicFormatConverter;
    if (!IsEqualGUID(wicSourceGUID, wicTargetGUID))
    {
        // Create a WIC format converter.
        hr = wicFactory->CreateFormatConverter(&wicFormatConverter);
        if (FAILED(hr))
        {
            LogError("wicFactory->CreateFormatConverter failed.");
            return false;
        }

        // Initialize the format converter to convert to the nearest format
        // supported by the encoder.
        hr = wicFormatConverter->Initialize(wicTextureBitmap, wicTargetGUID,
            WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom);
        if (FAILED(hr))
        {
            LogError("wicFormatConverter->Initialize failed.");
            return false;
        }

        // Use the format converter.
        wicBitmapSource = wicFormatConverter;
    }

    // Send the pixels to the encoder.
    hr = wicFrameEncode->WriteSource(wicBitmapSource, nullptr);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->WriteSource failed.");
        return false;
    }

    // Commit the frame.
    hr = wicFrameEncode->Commit();
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->Commit failed.");
        return false;
    }

    // Commit the encoder.
    hr = wicEncoder->Commit();
    if (FAILED(hr))
    {
        LogError("wicEncoder->Commit failed.");
        return false;
    }

    return true;
}

WICFileIO::ComInitializer::~ComInitializer()
{
    if (mInitialized)
    {
        ::CoUninitialize();
    }
}

WICFileIO::ComInitializer::ComInitializer()
{
    HRESULT hr = ::CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    mInitialized = SUCCEEDED(hr);
}

bool WICFileIO::ComInitializer::IsInitialized() const
{
    return mInitialized;
}

template <typename T>
WICFileIO::ComObject<T>::ComObject()
    :
    object(nullptr)
{
}

template <typename T>
WICFileIO::ComObject<T>::ComObject(T* inObject)
    :
    object(inObject)
{
    if (object != nullptr)
    {
        object->AddRef();
    }
}

template <typename T>
WICFileIO::ComObject<T>::~ComObject()
{
    if (object)
    {
        object->Release();
    }
}

template <typename T>
WICFileIO::ComObject<T>::operator T*() const
{
    return object;
}

template <typename T>
T& WICFileIO::ComObject<T>::operator*() const
{
    return *object;
}

template <typename T>
T** WICFileIO::ComObject<T>::operator&()
{
    return &object;
}

template <typename T>
T* WICFileIO::ComObject<T>::operator->() const
{
    return object;
}


WICFileIO::LoadFormatMap const WICFileIO::msLoadFormatMap[NUM_LOAD_FORMATS] =
{
    { DF_B5G6R5_UNORM, &GUID_WICPixelFormat16bppBGR565, nullptr },
    { DF_B5G5R5A1_UNORM, &GUID_WICPixelFormat16bppBGR555, nullptr },
    { DF_R10G10B10A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102, nullptr },
    { DF_R10G10B10_XR_BIAS_A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102XR, nullptr },
    { DF_R1_UNORM, &GUID_WICPixelFormatBlackWhite, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat2bppGray, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat4bppGray, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat8bppGray, nullptr },
    { DF_R16_UNORM, &GUID_WICPixelFormat16bppGray, nullptr },
    { DF_R32_FLOAT, &GUID_WICPixelFormat32bppGrayFloat, nullptr },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppRGBA, nullptr },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppBGRA, &GUID_WICPixelFormat32bppRGBA },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppRGBA, nullptr },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppBGRA, &GUID_WICPixelFormat64bppRGBA }

    // B8G8R8A8 is not supported for Texture2 in DX11.  We convert all
    // unmatched formats to R8G8B8A8.
    //{ DF_B8G8R8A8_UNORM, &GUID_WICPixelFormat32bppBGRA }
};

WICFileIO::SaveFormatMap const WICFileIO::msSaveFormatMap[NUM_SAVE_FORMATS] =
{
    { DF_B5G6R5_UNORM, &GUID_WICPixelFormat16bppBGR565 },
    { DF_B5G5R5A1_UNORM, &GUID_WICPixelFormat16bppBGR555 },
    { DF_R10G10B10A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102 },
    { DF_R10G10B10_XR_BIAS_A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102XR },
    { DF_R1_UNORM, &GUID_WICPixelFormatBlackWhite },
    { DF_R8_UNORM, &GUID_WICPixelFormat8bppGray },
    { DF_R16_UNORM, &GUID_WICPixelFormat16bppGray },
    { DF_R32_FLOAT, &GUID_WICPixelFormat32bppGrayFloat },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppRGBA },
    { DF_B8G8R8A8_UNORM, &GUID_WICPixelFormat32bppBGRA },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppRGBA }
};
