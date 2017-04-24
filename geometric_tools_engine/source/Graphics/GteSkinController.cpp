// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteSkinController.h>
#include <Graphics/GteNode.h>
#include <Graphics/GteVisual.h>
using namespace gte;

SkinController::~SkinController()
{
}

SkinController::SkinController(int numVertices, int numBones, Updater const& postUpdate)
    :
    mNumVertices(numVertices),
    mNumBones(numBones),
    mBones(numBones),
    mWeights(numBones, numVertices),
    mOffsets(numBones, numVertices),
    mPostUpdate(postUpdate),
    mPosition(nullptr),
    mStride(0),
    mFirstUpdate(true),
    mCanUpdate(false)
{
}

bool SkinController::Update(double applicationTime)
{
    if (!Controller::Update(applicationTime))
    {
        return false;
    }

    if (mFirstUpdate)
    {
        mFirstUpdate = false;
        OnFirstUpdate();
    }

    if (mCanUpdate)
    {
        // The skin vertices are calculated in the bone world coordinate system,
        // so the visual's world transform must be the identity.
        Visual* visual = reinterpret_cast<Visual*>(mObject);
        visual->worldTransform = Transform::IDENTITY;
        visual->worldTransformIsCurrent = true;

        // Compute the skin vertex locations.
        char* current = mPosition;
        for (int vertex = 0; vertex < mNumVertices; ++vertex)
        {
            Vector4<float> position{ 0.0f, 0.0f, 0.0f, 0.0f };
            for (int bone = 0; bone < mNumBones; ++bone)
            {
                float weight = mWeights[vertex][bone];
                if (weight != 0.0f)
                {
                    Vector4<float> offset = mOffsets[vertex][bone];
#if defined (GTE_USE_MAT_VEC)
                    Vector4<float> worldOffset =
                        mBones[bone].lock()->worldTransform * offset;
#else
                    Vector4<float> worldOffset =
                        offset * mBones[bone].lock()->worldTransform;
#endif
                    position += weight * worldOffset;
                }
            }

            Vector3<float>* target = reinterpret_cast<Vector3<float>*>(current);
            (*target)[0] = position[0];
            (*target)[1] = position[1];
            (*target)[2] = position[2];
            current += mStride;
        }

        visual->UpdateModelBound();
        visual->UpdateModelNormals();
        mPostUpdate(visual->GetVertexBuffer());
        return true;
    }

    return false;
}

void SkinController::OnFirstUpdate()
{
    // Get access to the vertex buffer positions to store the blended targets.
    Visual* visual = reinterpret_cast<Visual*>(mObject);
    VertexBuffer* vbuffer = visual->GetVertexBuffer().get();
    if (mNumVertices == static_cast<int>(vbuffer->GetNumElements()))
    {
        // Get the position data.
        VertexFormat vformat = vbuffer->GetFormat();
        int const numAttributes = vformat.GetNumAttributes();
        for (int i = 0; i < numAttributes; ++i)
        {
            VASemantic semantic;
            DFType type;
            unsigned int unit, offset;
            if (vformat.GetAttribute(i, semantic, type, unit, offset))
            {
                if (semantic == VA_POSITION
                    && (type == DF_R32G32B32_FLOAT || type == DF_R32G32B32A32_FLOAT))
                {
                    mPosition = vbuffer->GetData() + offset;
                    mStride = vformat.GetVertexSize();
                    mCanUpdate = true;
                    break;
                }
            }
        }
    }

    mCanUpdate = (mPosition != nullptr);
}
