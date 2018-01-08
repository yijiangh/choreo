// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.2 (2016/07/06)

#include <Graphics/GL4/GteOpenGL.h>
#include <GL/glx.h>
#include <Graphics/GL4/GL/glxext.h>

void* GetOpenGLFunctionPointer(char const* name)
{
    return (void*)(*glXGetProcAddress)((GLubyte const*)name);
}

//void InitializeGLX()
//{
//}

//template <typename PGLXFunction>
//static void GetGLXFunction(char const* name, PGLXFunction& function)
//{
//    function = (PGLXFunction)glXGetProcAddress(name);
//}

