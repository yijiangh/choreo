// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once

#include <Graphics/GteCamera.h>
#include <Graphics/GteNode.h>

namespace gte
{

class GTE_IMPEXP Trackball
{
public:
    // Construction.  The trackball is the largest circle centered in the
    // rectangle of dimensions xSize-by-ySize.  The rectangle is assumed to
    // be defined in right-handed coordinates, so if you use a window client
    // rectangle for the trackball and this rectangle is in left-handed
    // coordinates, you must reflect the y-values in SetInitialPoint and
    // SetFinalPoint by (ySize - 1 - y).  A root node is used to represent
    // the trackball orientation.  Objects may be attached and detached as
    // desired.
    Trackball();
    Trackball(int xSize, int ySize, std::shared_ptr<Camera> const& camera);

    // Member access.  The Set function is for deferred construction after
    // a default construction of a trackball.
    void Set(int xSize, int ySize, std::shared_ptr<Camera> const& camera);
    inline int GetXSize() const;
    inline int GetYSize() const;
    inline std::shared_ptr<Camera> const& GetCamera() const;
    inline std::shared_ptr<Node> const& GetRoot() const;
    inline Matrix4x4<float> const& GetOrientation() const;

    // The root node is the top-level node of a hierarchy whose local
    // transformation is the trackball orientation relative to the specified
    // camera.  The camera directions {D,U,R} act as the world coordinate
    // system.
    void Attach(std::shared_ptr<Spatial> const& object);
    void Detach(std::shared_ptr<Spatial> const& object);
    void DetachAll();
    inline void Update(double applicationTime = 0.0);

    // Set the arc on the sphere along which the trackball rotates.  The
    // standard use is to set the initial point via a mouse button click on a
    // window rectangle.  Mark the trackball as active and set final points
    // based on the locations of the dragged mouse.  Once the mouse button is
    // released, mark the trackball as inactive.
    inline void SetActive(bool active);
    inline bool GetActive() const;

    // Set the initial point of the arc.  The current trackball orientation is
    // recorded.  On construction, the initial point is set to the rectangle
    // center.
    void SetInitialPoint(int x, int y);

    // Set the final point of the arc.  The trackball orientation is updated
    // by the incremental rotation implied by the arc endpoints.
    void SetFinalPoint(int x, int y);

protected:
    void UpdateOrientation();

    int mXSize, mYSize;
    std::shared_ptr<Camera> mCamera;
    std::shared_ptr<Node> mRoot;
    Matrix4x4<float> mInitialOrientation;
    float mMultiplier, mX0, mY0, mX1, mY1;
    bool mActive, mValidTrackball;
};


inline int Trackball::GetXSize() const
{
    return mXSize;
}

inline int Trackball::GetYSize() const
{
    return mYSize;
}

inline std::shared_ptr<Camera> const& Trackball::GetCamera() const
{
    return mCamera;
}

inline std::shared_ptr<Node> const& Trackball::GetRoot() const
{
    return mRoot;
}

inline Matrix4x4<float> const& Trackball::GetOrientation() const
{
    return mRoot->worldTransform.GetRotation();
}

inline void Trackball::Update(double applicationTime)
{
    mRoot->Update(applicationTime);
}

inline void Trackball::SetActive(bool active)
{
    mActive = active;
}

inline bool Trackball::GetActive() const
{
    return mActive;
}


}
