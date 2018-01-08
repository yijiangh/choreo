// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteLightCameraGeometry.h>
using namespace gte;

LightCameraGeometry::LightCameraGeometry()
    :
    lightModelPosition({ 0.0f, 0.0f, 0.0f, 1.0f }),
    lightModelDirection({ 0.0f, 0.0f, -1.0f, 0.0f }),
    lightModelUp({ 0.0f, 1.0f, 0.0f, 0.0f }),
    lightModelRight({ 1.0f, 0.0f, 0.0f, 0.0f }),
    cameraModelPosition({ 0.0f, 0.0f, 0.0f, 1.0f }),
    cameraModelDirection({ 0.0f, 0.0f, -1.0f, 0.0f }),
    cameraModelUp({ 0.0f, 1.0f, 0.0f, 0.0f }),
    cameraModelRight({ 1.0f, 0.0f, 0.0f, 0.0f })
{
}
