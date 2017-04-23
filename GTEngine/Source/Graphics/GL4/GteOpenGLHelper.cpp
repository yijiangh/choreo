// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GL4/GteOpenGLHelper.h>


void OpenGLReportListener(char const* glFunction, GLenum code)
{
    std::string strFunction(glFunction);
    if (code != GL_ZERO)
    {
        std::string strCode;
        switch (code)
        {
        case GL_INVALID_ENUM:
            strCode = "GL_INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            strCode = "GL_INVALID_VALUE";
            break;
        case GL_INVALID_OPERATION:
            strCode = "GL_INVALID_OPERATION";
            break;
        case GL_STACK_OVERFLOW:
            strCode = "GL_STACK_OVERFLOW";
            break;
        case GL_STACK_UNDERFLOW:
            strCode = "GL_STACK_UNDERFLOW";
            break;
        case GL_OUT_OF_MEMORY:
            strCode = "GL_OUT_OF_MEMORY";
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            strCode = "GL_INVALID_FRAMEBUFFER_OPERATION";
            break;
        case GL_CONTEXT_LOST:
            strCode = "GL_CONTEXT_LOST";
            break;
        default:
            strCode = "unknown error";
            break;
        }
        LogWarning("GL error <" + strCode + "> in " + strFunction);
    }
    else
    {
        LogError("GL function " + strFunction + " is null.");
    }
}

