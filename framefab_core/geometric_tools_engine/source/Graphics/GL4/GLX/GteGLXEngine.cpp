// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/07/01)

#include <GTEnginePCH.h>
#include <Graphics/GL4/GLX/GteGLXEngine.h>
#include <Applications/GLX/GteWindowSystem.h>
#include <X11/Xlib.h>
#include <GL/glx.h>
using namespace gte;

GLXEngine::~GLXEngine()
{
    Terminate();
}

GLXEngine::GLXEngine(Display* display, unsigned long window, GLXContext context,
    int xSize, int ySize, bool saveDriverInfo, int requiredMajor, int requiredMinor)
    :
    GL4Engine(),
    mDisplay(display),
    mWindow(window),
    mImmediate(context),
    mIsComputeWindow(false)
{
    mXSize = xSize;
    mYSize = ySize;
    Initialize(requiredMajor, requiredMinor, saveDriverInfo);
}

GLXEngine::GLXEngine(bool saveDriverInfo, int requiredMajor, int requiredMinor)
    :
    GL4Engine(),
    mDisplay(nullptr),
    mWindow(0),
    mImmediate(nullptr),
    mIsComputeWindow(false)
{
    if (TheWindowSystem.Create(mDisplay, mImmediate, mWindow))
    {
        mIsComputeWindow = true;
        Initialize(requiredMajor, requiredMinor, saveDriverInfo);
    }
}

bool GLXEngine::IsActive() const
{
    return mImmediate == glXGetCurrentContext();
}

void GLXEngine::MakeActive()
{
    if (mImmediate != glXGetCurrentContext())
    {
        glXMakeCurrent(mDisplay, mWindow, mImmediate);
    }
}

void GLXEngine::DisplayColorBuffer(unsigned int syncInterval)
{
    // TODO: Disable vertical sync if possible.
    (void)syncInterval;

    glXSwapBuffers(mDisplay, mWindow);
}

bool GLXEngine::Initialize(int requiredMajor, int requiredMinor, bool saveDriverInfo)
{
    if (!glXMakeCurrent(mDisplay, mWindow, mImmediate))
    {
        LogError("glXMakeCurrent failed.");
        glXDestroyContext(mDisplay, mImmediate);
        mDisplay = nullptr;
        mWindow = 0;
        mImmediate = nullptr;
        return false;
    }

    // Get the function pointers for OpenGL; initialize the viewport,
    // default global state, and default font.
    return GL4Engine::Initialize(requiredMajor, requiredMinor, saveDriverInfo);
}

void GLXEngine::Terminate()
{
    GL4Engine::Terminate();

    if (mDisplay && mImmediate)
    {
        glXDestroyContext(mDisplay, mImmediate);
    }

    if (mIsComputeWindow)
    {
        XDestroyWindow(mDisplay, mWindow);
    }
}
