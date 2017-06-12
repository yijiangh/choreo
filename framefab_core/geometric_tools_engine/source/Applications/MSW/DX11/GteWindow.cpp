// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/07/16)

#include <GTEnginePCH.h>
#include <Applications/MSW/DX11/GteWindow.h>
using namespace gte;

Window::Parameters::Parameters()
    :
    deviceCreationFlags(0),
    featureLevel(D3D_FEATURE_LEVEL_11_0)
{
}

Window::Parameters::Parameters(std::wstring const& inTitle,
    int inXOrigin, int inYOrigin, int inXSize, int inYSize)
    :
    MSWWindow::Parameters(inTitle, inXOrigin, inYOrigin, inXSize, inYSize),
    deviceCreationFlags(0),
    featureLevel(D3D_FEATURE_LEVEL_11_0)
{
}

Window::Window(Parameters& parameters)
    :
    MSWWindow(parameters),
    mEngine(std::static_pointer_cast<GraphicsEngine>(mBaseEngine))
{
}

Window::~Window()
{
    if (mEngine)
    {
        auto dxengine = std::static_pointer_cast<DX11Engine>(mEngine);
        dxengine->ExitFullscreen();
    }
}
