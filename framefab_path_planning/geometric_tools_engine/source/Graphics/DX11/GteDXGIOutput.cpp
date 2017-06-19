// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/DX11/GteDXGIOutput.h>
using namespace gte;


DXGIOutput::~DXGIOutput()
{
    if (mOutput)
    {
        mOutput->Release();
    }
}

DXGIOutput::DXGIOutput(DXGIOutput const& object)
    :
    mOutput(nullptr)
{
    *this = object;
}

DXGIOutput::DXGIOutput(IDXGIOutput* output)
    :
    mOutput(output)
{
    ZeroMemory(&mDescription, sizeof(DXGI_OUTPUT_DESC));
    if (mOutput)
    {
        HRESULT hr = mOutput->GetDesc(&mDescription);
        CHECK_HR_RETURN_NONE("Unexpected error");
    }
}

DXGIOutput& DXGIOutput::operator=(DXGIOutput const& object)
{
    if (object.mOutput)
    {
        object.mOutput->AddRef();
    }

    SafeRelease(mOutput);

    mOutput = object.mOutput;
    mDescription = object.mDescription;
    return *this;
}

IDXGIOutput* DXGIOutput::GetOutput() const
{
    return mOutput;
}

DXGI_OUTPUT_DESC const& DXGIOutput::GetDescription() const
{
    return mDescription;
}

HRESULT DXGIOutput::GetDisplayModes(DXGI_FORMAT format,
    std::vector<DXGI_MODE_DESC>& modeDescriptions)
{
    modeDescriptions.clear();

    if (mOutput)
    {
        // The zero value for 'flags' says to return the maximum number of
        // modes, regardless of the DXGI_ENUM_MODES possibilities for flags.
        // We might want to allow for a different value for DX11.1 when
        // stereo modes are available.
        UINT const flags = 0;
        UINT numModes = 0;
        HRESULT hr = mOutput->GetDisplayModeList(format, flags, &numModes,
            nullptr);
        CHECK_HR_RETURN("Unexpected error", hr);

        if (numModes > 0)
        {
            modeDescriptions.resize(numModes);
            hr = mOutput->GetDisplayModeList(format, flags, &numModes,
                &modeDescriptions[0]);
            CHECK_HR_RETURN("Unexpected error", hr);
            return hr;
        }

        // No modes available for the requested format.  This is not an
        // error condition.  The caller must test that the array of
        // descriptions has no elements and choose another format that
        // might have modes associated with it.
        return S_OK;
    }

    LogError("Output not yet set.");
    return DXGI_ERROR_INVALID_CALL;
}

HRESULT DXGIOutput::FindClosestMatchingMode(DXGI_MODE_DESC const& requested,
    DXGI_MODE_DESC& closest)
{
    if (mOutput)
    {
        HRESULT hr = mOutput->FindClosestMatchingMode(&requested, &closest,
            nullptr);
        CHECK_HR_RETURN_NONE("Unexpected error");
        return hr;
    }

    LogError("Output not yet set.");
    return DXGI_ERROR_INVALID_CALL;
}

