// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/11/13)

#pragma once

#include <Mathematics/GteMatrix4x4.h>
#include <Graphics/GteTexture3.h>
#include <Graphics/GteVisualEffect.h>

namespace gte
{

class GTE_IMPEXP Texture3Effect : public VisualEffect
{
public:
    // Construction.
    Texture3Effect(std::shared_ptr<ProgramFactory> const& factory,
        std::shared_ptr<Texture3> const& texture, SamplerState::Filter filter,
        SamplerState::Mode mode0, SamplerState::Mode mode1,
        SamplerState::Mode mode2);

    // Member access.
    inline void SetPVWMatrix(Matrix4x4<float> const& pvwMatrix);
    inline Matrix4x4<float> const& GetPVWMatrix() const;

    // Required to bind and update resources.
    inline std::shared_ptr<ConstantBuffer> const& GetPVWMatrixConstant() const;
    inline std::shared_ptr<Texture3> const& GetTexture() const;
    inline std::shared_ptr<SamplerState> const& GetSampler() const;

    void SetPVWMatrixConstant(std::shared_ptr<ConstantBuffer> const& pvwMatrix);

private:
    // Vertex shader parameters.
    std::shared_ptr<ConstantBuffer> mPVWMatrixConstant;

    // Pixel shader parameters.
    std::shared_ptr<Texture3> mTexture;
    std::shared_ptr<SamplerState> mSampler;

    // Convenience pointer.
    Matrix4x4<float>* mPVWMatrix;

    // Shader source code as strings.
    static std::string const msGLSLVSSource;
    static std::string const msGLSLPSSource;
    static std::string const msHLSLSource;
    static std::string const* msVSSource[ProgramFactory::PF_NUM_API];
    static std::string const* msPSSource[ProgramFactory::PF_NUM_API];
};


inline void Texture3Effect::SetPVWMatrix(Matrix4x4<float> const& pvwMatrix)
{
    *mPVWMatrix = pvwMatrix;
}

inline Matrix4x4<float> const& Texture3Effect::GetPVWMatrix() const
{
    return *mPVWMatrix;
}

inline std::shared_ptr<ConstantBuffer> const&
Texture3Effect::GetPVWMatrixConstant() const
{
    return mPVWMatrixConstant;
}

inline std::shared_ptr<Texture3> const& Texture3Effect::GetTexture() const
{
    return mTexture;
}

inline std::shared_ptr<SamplerState> const& Texture3Effect::GetSampler() const
{
    return mSampler;
}

}
