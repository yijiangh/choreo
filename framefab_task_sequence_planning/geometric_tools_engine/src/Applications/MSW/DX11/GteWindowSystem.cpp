// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Applications/MSW/DX11/GteWindow.h>
#include <Applications/MSW/DX11/GteWindowSystem.h>
#include <Graphics/DX11/GteDX11Engine.h>
#include <Graphics/DX11/GteHLSLProgramFactory.h>
using namespace gte;

WindowSystem gte::TheWindowSystem;

WindowSystem::~WindowSystem()
{
}

WindowSystem::WindowSystem()
{
}

void WindowSystem::CreateEngineAndProgramFactory(MSWWindow::Parameters& parameters)
{
    Window::Parameters& p = static_cast<Window::Parameters&>(parameters);

    auto engine = std::make_shared<DX11Engine>(nullptr, p.handle, p.xSize,
        p.ySize, D3D_DRIVER_TYPE_HARDWARE, nullptr, p.deviceCreationFlags,
        p.featureLevel);

    if (engine->GetDevice())
    {
        parameters.engine = engine;
        parameters.factory = std::make_shared<HLSLProgramFactory>();
        parameters.created = true;
    }
    else
    {
        LogError("Cannot create graphics engine.");
        parameters.engine = nullptr;
        parameters.factory = nullptr;
        parameters.created = false;
    }
}
