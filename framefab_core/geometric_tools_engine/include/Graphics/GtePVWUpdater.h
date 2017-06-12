// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Graphics/GteCamera.h>
#include <Graphics/GteConstantBuffer.h>
#include <map>

namespace gte
{

class GTE_IMPEXP PVWUpdater
{
public:
    // Construction and destruction.
    virtual ~PVWUpdater();
    PVWUpdater();
    PVWUpdater(std::shared_ptr<Camera> const& camera, BufferUpdater const& updater);

    // Member access.  The functions are for deferred construction after
    // a default construction of a PVWUpdater.
    void Set(std::shared_ptr<Camera> const& camera, BufferUpdater const& updater);
    inline std::shared_ptr<Camera> const& GetCamera() const;
    inline void SetUpdater(BufferUpdater const& updater);
    inline BufferUpdater const& GetUpdater() const;

    // Update the constant buffer's projection-view-world matrix (pvw-matrix)
    // when the camera's view or projection matrices change.  The input
    // 'pvwMatrixName' is the name specified in the shader program and is
    // used in calls to ConstantBuffer::SetMember<Matrix4x4<float>>(...).
    // If you modify the view or projection matrices directly through the
    // Camera interface, you are responsible for calling UpdatePVWMatrices().
    //
    // The Subscribe function uses the address of 'worldMatrix' as a key
    // to a std::map, so be careful to ensure that 'worldMatrix' persists
    // until a call to an Unsubscribe function.  The return value of Subscribe
    // is 'true' as long as 'cbuffer' is not already subscribed and actually
    // has a member named 'pvwMatrixName'.  The return value of Unsubscribe is
    // true if and only if the input matrix is currently subscribed.
    bool Subscribe(Matrix4x4<float> const& worldMatrix,
        std::shared_ptr<ConstantBuffer> const& cbuffer,
        std::string const& pvwMatrixName = "pvwMatrix");
    bool Unsubscribe(Matrix4x4<float> const& worldMatrix);
    void UnsubscribeAll();

    // After any camera modifictions that change the projection or view
    // matrices, call this function to update the constant buffers that
    // are subscribed.
    void Update();

protected:
    std::shared_ptr<Camera> mCamera;
    BufferUpdater mUpdater;

    typedef Matrix4x4<float> const* PVWKey;
    typedef std::pair<std::shared_ptr<ConstantBuffer>, std::string> PVWValue;
    std::map<PVWKey, PVWValue> mSubscribers;
};


inline std::shared_ptr<Camera> const& PVWUpdater::GetCamera() const
{
    return mCamera;
}

inline void PVWUpdater::SetUpdater(BufferUpdater const& updater)
{
    mUpdater = updater;
}

inline BufferUpdater const& PVWUpdater::GetUpdater() const
{
    return mUpdater;
}

}
