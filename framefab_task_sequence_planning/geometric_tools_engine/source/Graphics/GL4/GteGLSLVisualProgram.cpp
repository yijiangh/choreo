// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GL4/GteGLSLVisualProgram.h>
using namespace gte;

GLSLVisualProgram::~GLSLVisualProgram()
{
    if (glIsProgram(mProgramHandle))
    {
        if (glIsShader(mVShaderHandle))
        {
            glDetachShader(mProgramHandle, mVShaderHandle);
            glDeleteShader(mVShaderHandle);
        }

        if (glIsShader(mPShaderHandle))
        {
            glDetachShader(mProgramHandle, mPShaderHandle);
            glDeleteShader(mPShaderHandle);
        }

        if (glIsShader(mGShaderHandle))
        {
            glDetachShader(mProgramHandle, mGShaderHandle);
            glDeleteShader(mGShaderHandle);
        }

        glDeleteProgram(mProgramHandle);
    }
}

GLSLVisualProgram::GLSLVisualProgram(GLuint programHandle,
    GLuint vshaderHandle, GLuint pshaderHandle, GLuint gshaderHandle)
    :
    mProgramHandle(programHandle),
    mVShaderHandle(vshaderHandle),
    mPShaderHandle(pshaderHandle),
    mGShaderHandle(gshaderHandle),
    mReflector(programHandle)
{
}
