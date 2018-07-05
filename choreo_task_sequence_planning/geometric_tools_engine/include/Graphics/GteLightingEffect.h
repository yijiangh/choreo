// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/11/13)

#pragma once

#include <Graphics/GteVisualEffect.h>
#include <Graphics/GteMaterial.h>
#include <Graphics/GteLighting.h>
#include <Graphics/GteLightCameraGeometry.h>

namespace gte
{

class GTE_IMPEXP LightingEffect : public VisualEffect
{
protected:
    // Construction (abstract base class).  The shader source code string
    // arrays must contain strings for any supported graphics API.
    LightingEffect(std::shared_ptr<ProgramFactory> const& factory,
        BufferUpdater const& updater, std::string const* vsSource[],
        std::string const* psSource[], std::shared_ptr<Material> const& material,
        std::shared_ptr<Lighting> const& lighting,
        std::shared_ptr<LightCameraGeometry> const& geometry);

public:
    // Member access.
    inline void SetMaterial(std::shared_ptr<Material> const& material);
    inline void SetLighting(std::shared_ptr<Lighting> const& lighting);
    inline void SetGeometry(std::shared_ptr<LightCameraGeometry> const& geometry);
    inline std::shared_ptr<Material> const& GetMaterial() const;
    inline std::shared_ptr<Lighting> const& GetLighting() const;
    inline std::shared_ptr<LightCameraGeometry> const& GetGeometry() const;
    inline std::shared_ptr<ConstantBuffer> const& GetPVWMatrixConstant() const;
    inline std::shared_ptr<ConstantBuffer> const& GetMaterialConstant() const;
    inline std::shared_ptr<ConstantBuffer> const& GetLightingConstant() const;
    inline std::shared_ptr<ConstantBuffer> const& GetGeometryConstant() const;

    void SetPVWMatrixConstant(std::shared_ptr<ConstantBuffer> const& pvwMatrix);

    // After you set or modify 'material', 'light', or 'geometry', call the update
    // to inform any listener that the corresponding constant buffer has changed.
    // The derived classes construct the constant buffers to store the minimal
    // information from Material, Light, or Camera.  The pvw-matrix constant update
    // requires knowledge of the world transform of the object to which the effect
    // is attached, so its update must occur outside of this class.  Derived
    // classes update the system memory of the constant buffers and the base class
    // updates video memory.
    virtual void UpdateMaterialConstant();
    virtual void UpdateLightingConstant();
    virtual void UpdateGeometryConstant();

protected:
    std::shared_ptr<Material> mMaterial;
    std::shared_ptr<Lighting> mLighting;
    std::shared_ptr<LightCameraGeometry> mGeometry;

    std::shared_ptr<ConstantBuffer> mPVWMatrixConstant;

    // The derived-class constructors are responsible for creating these
    // according to their needs.
    std::shared_ptr<ConstantBuffer> mMaterialConstant;
    std::shared_ptr<ConstantBuffer> mLightingConstant;
    std::shared_ptr<ConstantBuffer> mGeometryConstant;

    // HLSL has a shader intrinsic lit() function.
    // This inline string of code reproduces that function for GLSL.
    // Static method used here because this string needs to be generated
    // before code (which may also be in global initializers) tries to use it.
    static std::string GetShaderSourceLitFunctionGLSL();
};

inline void LightingEffect::SetMaterial(std::shared_ptr<Material> const& material)
{
    mMaterial = material;
}

inline void LightingEffect::SetLighting(std::shared_ptr<Lighting> const& lighting)
{
    mLighting = lighting;
}

inline void LightingEffect::SetGeometry(std::shared_ptr<LightCameraGeometry> const& geometry)
{
    mGeometry = geometry;
}

inline std::shared_ptr<Material> const& LightingEffect::GetMaterial() const
{
    return mMaterial;
}

inline std::shared_ptr<Lighting> const& LightingEffect::GetLighting() const
{
    return mLighting;
}

inline std::shared_ptr<LightCameraGeometry> const& LightingEffect::GetGeometry() const
{
    return mGeometry;
}

inline std::shared_ptr<ConstantBuffer> const& LightingEffect::GetPVWMatrixConstant() const
{
    return mPVWMatrixConstant;
}

inline std::shared_ptr<ConstantBuffer> const& LightingEffect::GetMaterialConstant() const
{
    return mMaterialConstant;
}

inline std::shared_ptr<ConstantBuffer> const& LightingEffect::GetLightingConstant() const
{
    return mLightingConstant;
}

inline std::shared_ptr<ConstantBuffer> const& LightingEffect::GetGeometryConstant() const
{
    return mGeometryConstant;
}

}
