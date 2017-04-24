// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GtePixelShader.h>
using namespace gte;

#if defined(GTE_DEV_OPENGL)
PixelShader::PixelShader(GLSLReflection const& reflector)
    :
    Shader(reflector, GLSLReflection::ST_PIXEL)
{
    mType = GT_PIXEL_SHADER;
}
#else
PixelShader::PixelShader(HLSLShader const& program)
    :
    Shader(program)
{
    mType = GT_PIXEL_SHADER;
}
#endif
