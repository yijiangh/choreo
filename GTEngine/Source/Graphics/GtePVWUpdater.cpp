// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GtePVWUpdater.h>
using namespace gte;

PVWUpdater::~PVWUpdater()
{
}

PVWUpdater::PVWUpdater()
{
    Set(nullptr, [](std::shared_ptr<Buffer> const&) {});
}

PVWUpdater::PVWUpdater(std::shared_ptr<Camera> const& camera, BufferUpdater const& updater)
{
    Set(camera, updater);
}

void PVWUpdater::Set(std::shared_ptr<Camera> const& camera, BufferUpdater const& updater)
{
    mCamera = camera;
    mUpdater = updater;
}

bool PVWUpdater::Subscribe(Matrix4x4<float> const& worldMatrix,
    std::shared_ptr<ConstantBuffer> const& cbuffer,
    std::string const& pvwMatrixName)
{
    if (cbuffer && cbuffer->HasMember(pvwMatrixName))
    {
        if (mSubscribers.find(&worldMatrix) == mSubscribers.end())
        {
            mSubscribers.insert(std::make_pair(&worldMatrix,
                std::make_pair(cbuffer, pvwMatrixName)));
            return true;
        }
    }
    return false;
}

bool PVWUpdater::Unsubscribe(Matrix4x4<float> const& worldMatrix)
{
    return mSubscribers.erase(&worldMatrix) > 0;
}

void PVWUpdater::UnsubscribeAll()
{
    mSubscribers.clear();
}

void PVWUpdater::Update()
{
    // The function is called knowing that mCamera is not null.
    Matrix4x4<float> pvMatrix = mCamera->GetProjectionViewMatrix();
    for (auto& element : mSubscribers)
    {
        // Compute the new projection-view-world matrix.  The matrix
        // *element.first is the model-to-world matrix for the associated
        // object.
#if defined(GTE_USE_MAT_VEC)
        Matrix4x4<float> pvwMatrix = pvMatrix * (*element.first);
#else
        Matrix4x4<float> pvwMatrix = (*element.first) * pvMatrix;
#endif
        // Copy the source matrix into the system memory of the constant
        // buffer.
        element.second.first->SetMember(element.second.second, pvwMatrix);

        // Allow the caller to update GPU memory as desired.
        mUpdater(element.second.first);
    }
}
