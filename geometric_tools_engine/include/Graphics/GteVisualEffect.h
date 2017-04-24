// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Graphics/GteProgramFactory.h>

namespace gte
{

class GTE_IMPEXP VisualEffect
{
public:
    // Construction and destruction.
    virtual ~VisualEffect();
    VisualEffect(std::shared_ptr<VisualProgram> const& program);

    // Member access.
    inline std::shared_ptr<VisualProgram> const& GetProgram() const;
    inline std::shared_ptr<VertexShader> const& GetVertexShader() const;
    inline std::shared_ptr<PixelShader> const& GetPixelShader() const;
    inline std::shared_ptr<GeometryShader> const& GetGeometryShader() const;

protected:
    // For derived classes to defer construction because they want to create
    // programs via a factory.
    VisualEffect();

    std::shared_ptr<VisualProgram> mProgram;
    BufferUpdater mBufferUpdater;
    TextureUpdater mTextureUpdater;
    TextureArrayUpdater mTextureArrayUpdater;
};


inline std::shared_ptr<VisualProgram> const& VisualEffect::GetProgram() const
{
    return mProgram;
}

inline std::shared_ptr<VertexShader> const& VisualEffect::GetVertexShader() const
{
    return mProgram->GetVShader();
}

inline std::shared_ptr<PixelShader> const& VisualEffect::GetPixelShader() const
{
    return mProgram->GetPShader();
}

inline std::shared_ptr<GeometryShader> const& VisualEffect::GetGeometryShader() const
{
    return mProgram->GetGShader();
}

}
