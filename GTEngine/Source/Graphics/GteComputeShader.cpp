// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteComputeShader.h>
using namespace gte;

#if defined(GTE_DEV_OPENGL)
ComputeShader::ComputeShader(GLSLReflection const& reflector)
    :
    Shader(reflector, GLSLReflection::ST_COMPUTE)
{
    mType = GT_COMPUTE_SHADER;
}
#else
ComputeShader::ComputeShader(HLSLShader const& program)
    :
    Shader(program)
{
    mType = GT_COMPUTE_SHADER;
}
#endif
