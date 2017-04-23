// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/DX11/GteDX11PerformanceCounter.h>
using namespace gte;


DX11PerformanceCounter::~DX11PerformanceCounter()
{
    FinalRelease(mFrequencyQuery);
    FinalRelease(mStartTimeQuery);
    FinalRelease(mFinalTimeQuery);
}

DX11PerformanceCounter::DX11PerformanceCounter(ID3D11Device* device)
    :
    mFrequencyQuery(nullptr),
    mStartTimeQuery(nullptr),
    mFinalTimeQuery(nullptr),
    mFrequency(0.0),
    mInvFrequency(0.0),
    mStartTime(0),
    mFinalTime(0),
    mTotalSeconds(0.0),
    mNumMeasurements(0)
{
    if (!device)
    {
        LogError("Input device is null.");
        return;
    }

    mTimeStamp.Frequency = 0;
    mTimeStamp.Disjoint = FALSE;

    D3D11_QUERY_DESC desc;
    desc.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
    desc.MiscFlags = D3D11_QUERY_MISC_NONE;
    HRESULT hr = device->CreateQuery(&desc, &mFrequencyQuery);
    CHECK_HR_RETURN_VOID("Failed to create frequency query");

    mFrequency = static_cast<double>(mTimeStamp.Frequency);
    mInvFrequency = 1.0/mFrequency;

    desc.Query = D3D11_QUERY_TIMESTAMP;
    desc.MiscFlags = D3D11_QUERY_MISC_NONE;
    hr = device->CreateQuery(&desc, &mStartTimeQuery);
    if (FAILED(hr))
    {
        SafeRelease(mFrequencyQuery);
        LogError("Failed to create start time query.");
        return;
    }

    hr = device->CreateQuery(&desc, &mFinalTimeQuery);
    if (FAILED(hr))
    {
        SafeRelease(mFrequencyQuery);
        SafeRelease(mStartTimeQuery);
        LogError("Failed to create final time query.");
        return;
    }
}

int64_t DX11PerformanceCounter::GetTicks() const
{
    return (mFinalTime >= mStartTime ? mFinalTime - mStartTime : 0);
}

double DX11PerformanceCounter::GetSeconds() const
{
    if (FALSE == mTimeStamp.Disjoint)
    {
        double numer = static_cast<double>(GetTicks());
        double denom = static_cast<double>(mTimeStamp.Frequency);
        return numer/denom;
    }
    else
    {
        return 0.0;
    }
}

double DX11PerformanceCounter::GetSeconds(int64_t numTicks) const
{
    return mInvFrequency*static_cast<double>(numTicks);
}

int64_t DX11PerformanceCounter::GetTicks(double seconds) const
{
    return static_cast<int64_t>(seconds*mFrequency);
}

void DX11PerformanceCounter::ResetAccumulateTime()
{
    mTotalSeconds = 0.0;
    mNumMeasurements = 0;
}

void DX11PerformanceCounter::AccumulateTime()
{
    mTotalSeconds += GetSeconds();
    ++mNumMeasurements;
}

double DX11PerformanceCounter::GetAverageSeconds() const
{
    if (mNumMeasurements > 0)
    {
        return mTotalSeconds/static_cast<double>(mNumMeasurements);
    }
    return 0.0;
}

unsigned int DX11PerformanceCounter::GetNumMeasurements() const
{
    return mNumMeasurements;
}

