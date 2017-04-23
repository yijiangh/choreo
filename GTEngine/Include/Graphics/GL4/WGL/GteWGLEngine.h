// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/06/30)

#pragma once

#include <Graphics/GL4/GteGL4Engine.h>

namespace gte
{

class GTE_IMPEXP WGLEngine : public GL4Engine
{
public:
    // Construction and destruction.  The first constructor is for windowed
    // graphics applications.  The second constructor is for windowless
    // compute-program applications.
    //
    // TODO: Currently, OpenGL 4.3 is required for compute shaders.  See
    // the comment in GteGL4Engine.h for 'MeetsRequirements()'.
    virtual ~WGLEngine();
    WGLEngine(HWND handle, bool saveDriverInfo, int requiredMajor = 4, int requiredMinor = 3);
    WGLEngine(bool saveDriverInfo, int requiredMajor = 4, int requiredMinor = 3);

    // Access to members that correspond to constructor inputs.
    inline HDC GetDevice() const;
    inline HGLRC GetImmediate() const;

    // Allow the user to switch between OpenGL contexts when there are
    // multiple instances of GL4Engine in an application.
    virtual bool IsActive() const override;
    virtual void MakeActive() override;

    // Support for clearing the color, depth, and stencil back buffers.
    virtual void DisplayColorBuffer(unsigned int syncInterval) override;

private:
    // Helpers for construction and destruction.
    virtual bool Initialize(int requiredMajor, int requiredMinor, bool saveDriverInfo) override;
    void Terminate();

    // Inputs to the constructor.
    HWND mHandle;

    // Objects created by the constructors.
    HDC mDevice;
    HGLRC mImmediate;
    std::wstring mComputeWindowClass;
    ATOM mComputeWindowAtom;
};

inline HDC WGLEngine::GetDevice() const
{
    return mDevice;
}

inline HGLRC WGLEngine::GetImmediate() const
{
    return mImmediate;
}

}
