// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteMaterial.h>
using namespace gte;

Material::Material()
    :
    emissive({ 0.0f, 0.0f, 0.0f, 1.0f }),
    ambient({ 0.0f, 0.0f, 0.0f, 1.0f }),
    diffuse({ 0.0f, 0.0f, 0.0f, 1.0f }),
    specular({ 0.0f, 0.0f, 0.0f, 1.0f })
{
}
