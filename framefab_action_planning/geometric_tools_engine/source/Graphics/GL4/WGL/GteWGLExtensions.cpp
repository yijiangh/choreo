// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/07/06)

#include <Graphics/GL4/GteOpenGL.h>
#include <Graphics/GL4/GL/wglext.h>
#include <cassert>

void* GetOpenGLFunctionPointer(char const* name)
{
    return wglGetProcAddress(name);
}

template <typename PWGLFunction>
static void GetWGLFunction(char const* name, PWGLFunction& function)
{
    function = (PWGLFunction)wglGetProcAddress(name);
}


static PFNWGLSWAPINTERVALEXTPROC swglSwapIntervalEXT = nullptr;
static PFNWGLGETSWAPINTERVALEXTPROC swglGetSwapIntervalEXT = nullptr;

int __stdcall wglSwapIntervalEXT(int interval)
{
    int result;
    if (swglSwapIntervalEXT)
    {
        result = swglSwapIntervalEXT(interval);
    }
    else
    {
        assert(false);
        result = 0;
    }
    return result;
}

int __stdcall wglGetSwapIntervalEXT(void)
{
    int result;
    if (swglGetSwapIntervalEXT)
    {
        result = swglGetSwapIntervalEXT();
    }
    else
    {
        assert(false);
        result = 0;
    }
    return result;
}

void InitializeWGL()
{
    GetWGLFunction("wglSwapIntervalEXT", swglSwapIntervalEXT);
    GetWGLFunction("wglGetSwapIntervalEXT", swglGetSwapIntervalEXT);
}

