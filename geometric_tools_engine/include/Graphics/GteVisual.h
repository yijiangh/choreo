// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Graphics/GteIndexBuffer.h>
#include <Graphics/GteSpatial.h>
#include <Graphics/GteVertexBuffer.h>
#include <Graphics/GteVisualEffect.h>

namespace gte
{

class GTE_IMPEXP Visual : public Spatial
{
public:
    // Construction and destruction.
    virtual ~Visual();
    Visual(
        std::shared_ptr<VertexBuffer> const& vbuffer =
            std::shared_ptr<VertexBuffer>(),
        std::shared_ptr<IndexBuffer> const& ibuffer =
            std::shared_ptr<IndexBuffer>(),
        std::shared_ptr<VisualEffect> const& effect =
            std::shared_ptr<VisualEffect>());

    // Member access.
    inline void SetVertexBuffer(std::shared_ptr<VertexBuffer> const& vbuffer);
    inline void SetIndexBuffer(std::shared_ptr<IndexBuffer> const& ibuffer);
    inline void SetEffect(std::shared_ptr<VisualEffect> const& effect);
    inline std::shared_ptr<VertexBuffer> const& GetVertexBuffer() const;
    inline std::shared_ptr<IndexBuffer> const& GetIndexBuffer() const;
    inline std::shared_ptr<VisualEffect> const& GetEffect() const;

    // Support for geometric updates.
    bool UpdateModelBound();
    bool UpdateModelNormals();

    // Public member access.
    BoundingSphere modelBound;

protected:
    // Support for geometric updates.
    inline virtual void UpdateWorldBound();

    // Support for hierarchical culling.
    inline virtual void GetVisibleSet(Culler& culler,
        std::shared_ptr<Camera> const& camera, bool noCull);

    std::shared_ptr<VertexBuffer> mVBuffer;
    std::shared_ptr<IndexBuffer> mIBuffer;
    std::shared_ptr<VisualEffect> mEffect;
};


inline void Visual::SetVertexBuffer(
    std::shared_ptr<VertexBuffer> const& vbuffer)
{
    mVBuffer = vbuffer;
}

inline void Visual::SetIndexBuffer(
    std::shared_ptr<IndexBuffer> const& ibuffer)
{
    mIBuffer = ibuffer;
}

inline void Visual::SetEffect(std::shared_ptr<VisualEffect> const& effect)
{
    mEffect = effect;
}

inline std::shared_ptr<VertexBuffer> const& Visual::GetVertexBuffer() const
{
    return mVBuffer;
}

inline std::shared_ptr<IndexBuffer> const& Visual::GetIndexBuffer() const
{
    return mIBuffer;
}

inline std::shared_ptr<VisualEffect> const& Visual::GetEffect() const
{
    return mEffect;
}

inline void Visual::UpdateWorldBound()
{
    modelBound.TransformBy(worldTransform, worldBound);
}

inline void Visual::GetVisibleSet(Culler& culler,
    std::shared_ptr<Camera> const&, bool)
{
    culler.Insert(this);
}


}
