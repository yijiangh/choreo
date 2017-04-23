// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Applications/GteWindow2.h>
#include <Imagics/GteImageUtility2.h>
using namespace gte;

Window2::Window2(Parameters& parameters)
    :
    Window(parameters),
    mClampToWindow(true),
    mDoFlip(false),
    mScreenTextureNeedsUpdate(false)
{
    if (parameters.allowResize)
    {
        parameters.created = false;
        return;
    }

    mOverlay = std::make_shared<OverlayEffect>(mProgramFactory, mXSize, mYSize, mXSize, mYSize,
        SamplerState::MIN_P_MAG_P_MIP_P, SamplerState::CLAMP, SamplerState::CLAMP, true);

    mScreenTexture = std::make_shared<Texture2>(DF_R8G8B8A8_UNORM, mXSize, mYSize);
    mScreenTexture->SetUsage(Resource::DYNAMIC_UPDATE);
    mOverlay->SetTexture(mScreenTexture);

    // The default is to disable depth and stenciling.  For layered drawing in
    // the z-direction, an application can choose to restore the default mode
    // of depth and stenciling turned on.
    mNoDepthStencilState = std::make_shared<DepthStencilState>();
    mNoDepthStencilState->depthEnable = false;
    mNoDepthStencilState->stencilEnable = false;
    mEngine->SetDepthStencilState(mNoDepthStencilState);

    // Callback function for ImageUtility2 functions.
    mDrawPixel = [this](int x, int y) { SetPixel(x, y, mPixelColor); };
    mPixelColor = 0;
}

bool Window2::OnResize(int, int)
{
    // See the comments in GteWindow.h.
    return false;
}

void Window2::OnDisplay()
{
    if (mScreenTextureNeedsUpdate)
    {
        mEngine->Update(mScreenTexture);
        mScreenTextureNeedsUpdate = false;
    }

    mEngine->Draw(mOverlay);
    DrawScreenOverlay();
    mEngine->DisplayColorBuffer(0);
}

void Window2::DrawScreenOverlay()
{
    // Stub for derived classes.
}

void Window2::ClearScreen(unsigned int color)
{
    unsigned int const numTexels = mScreenTexture->GetNumElements();
    unsigned int* texels = mScreenTexture->Get<unsigned int>();
    for (unsigned int i = 0; i < numTexels; ++i)
    {
        *texels++ = color;
    }
}

void Window2::SetPixel(int x, int y, unsigned int color)
{
    if (mClampToWindow)
    {
        if (x < 0 || x >= mXSize || y < 0 || y >= mYSize)
        {
            return;
        }
    }

    if (mDoFlip)
    {
        y = mYSize - 1 - y;
    }

    mScreenTexture->Get<unsigned int>()[x + mXSize * y] = color;
}

unsigned int Window2::GetPixel(int x, int y)
{
    if (mClampToWindow)
    {
        if (x < 0 || x >= mXSize || y < 0 || y >= mYSize)
        {
            return 0;
        }
    }

    if (mDoFlip)
    {
        y = mYSize - 1 - y;
    }

    return mScreenTexture->Get<unsigned int>()[x + mXSize * y];
}

void Window2::DrawThickPixel(int x, int y, int thick, unsigned int color)
{
    mPixelColor = color;
    ImageUtility2::DrawThickPixel(x, y, thick, mDrawPixel);
}

void Window2::DrawLine(int x0, int y0, int x1, int y1, unsigned int color)
{
    mPixelColor = color;
    ImageUtility2::DrawLine(x0, y0, x1, y1, mDrawPixel);
}

void Window2::DrawRectangle(int xMin, int yMin, int xMax, int yMax, unsigned int color, bool solid)
{
    mPixelColor = color;
    ImageUtility2::DrawRectangle(xMin, yMin, xMax, yMax, solid, mDrawPixel);
}

void Window2::DrawCircle(int xCenter, int yCenter, int radius, unsigned int color, bool solid)
{
    mPixelColor = color;
    ImageUtility2::DrawCircle(xCenter, yCenter, radius, solid, mDrawPixel);
}

void Window2::DrawEllipse(int xCenter, int yCenter, int xExtent, int yExtent, unsigned int color)
{
    mPixelColor = color;
    ImageUtility2::DrawEllipse(xCenter, yCenter, xExtent, yExtent, mDrawPixel);
}

void Window2::DrawFloodFill4(int x, int y, unsigned int foreColor, unsigned int backColor)
{
    ImageUtility2::DrawFloodFill4<unsigned int>(x, y, mXSize, mYSize, foreColor, backColor,
        [this](int x, int y, unsigned int color) { SetPixel(x, y, color); },
        [this](int x, int y) { return GetPixel(x, y); });
}
