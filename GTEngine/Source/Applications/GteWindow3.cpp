// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Applications/GteWindow3.h>
using namespace gte;

Window3::Window3(Parameters& parameters)
    :
    Window(parameters),
    mUpdater([this](std::shared_ptr<Buffer> const& buffer){ mEngine->Update(buffer); }),
#if defined(GTE_DEV_OPENGL)
    mCamera(std::make_shared<Camera>(true, false)),
#else
    mCamera(std::make_shared<Camera>(true, true)),
#endif
    mCameraRig(mCamera, 0.0f, 0.0f),
    mPVWMatrices(mCamera, mUpdater),
    mTrackball(mXSize, mYSize, mCamera)
{
    mCameraRig.RegisterMoveForward(KEY_UP);
    mCameraRig.RegisterMoveBackward(KEY_DOWN);
    mCameraRig.RegisterMoveUp(KEY_HOME);
    mCameraRig.RegisterMoveDown(KEY_END);
    mCameraRig.RegisterMoveRight(KEY_INSERT);
    mCameraRig.RegisterMoveLeft(KEY_DELETE);
    mCameraRig.RegisterTurnRight(KEY_RIGHT);
    mCameraRig.RegisterTurnLeft(KEY_LEFT);
    mCameraRig.RegisterLookUp(KEY_PAGE_UP);
    mCameraRig.RegisterLookDown(KEY_PAGE_DOWN);
}

void Window3::InitializeCamera(float upFovDegrees, float aspectRatio, float dmin, float dmax,
    float translationSpeed, float rotationSpeed, std::array<float, 3> const& pos,
    std::array<float, 3> const& dir, std::array<float, 3> const& up)
{
    mCamera->SetFrustum(upFovDegrees, aspectRatio, dmin, dmax);
    Vector4<float> camPosition{ pos[0], pos[1], pos[2], 1.0f };
    Vector4<float> camDVector{ dir[0], dir[1], dir[2], 0.0f };
    Vector4<float> camUVector{ up[0], up[1], up[2], 0.0f };
    Vector4<float> camRVector = Cross(camDVector, camUVector);
    mCamera->SetFrame(camPosition, camDVector, camUVector, camRVector);

    mCameraRig.ComputeWorldAxes();
    mCameraRig.SetTranslationSpeed(translationSpeed);
    mCameraRig.SetRotationSpeed(rotationSpeed);
}

bool Window3::OnResize(int xSize, int ySize)
{
    if (Window::OnResize(xSize, ySize))
    {
        float upFovDegrees, aspectRatio, dMin, dMax;
        mCamera->GetFrustum(upFovDegrees, aspectRatio, dMin, dMax);
        mCamera->SetFrustum(upFovDegrees, GetAspectRatio(), dMin, dMax);
        mPVWMatrices.Update();
        return true;
    }
    return false;
}

bool Window3::OnCharPress(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 't':  // Slower camera translation.
        mCameraRig.SetTranslationSpeed(0.5f * mCameraRig.GetTranslationSpeed());
        return true;

    case 'T':  // Faster camera translation.
        mCameraRig.SetTranslationSpeed(2.0f * mCameraRig.GetTranslationSpeed());
        return true;

    case 'r':  // Slower camera rotation.
        mCameraRig.SetRotationSpeed(0.5f * mCameraRig.GetRotationSpeed());
        return true;

    case 'R':  // Faster camera rotation.
        mCameraRig.SetRotationSpeed(2.0f * mCameraRig.GetRotationSpeed());
        return true;
    }

    return Window::OnCharPress(key, x, y);
}

bool Window3::OnKeyDown(int key, int, int)
{
    return mCameraRig.PushMotion(key);
}

bool Window3::OnKeyUp(int key, int, int)
{
    return mCameraRig.PopMotion(key);
}

bool Window3::OnMouseClick(MouseButton button, MouseState state, int x, int y,
    unsigned int)
{
    if (button == MOUSE_LEFT)
    {
        if (state == MOUSE_DOWN)
        {
            mTrackball.SetActive(true);
            mTrackball.SetInitialPoint(x, mYSize - 1 - y);
        }
        else
        {
            mTrackball.SetActive(false);
        }

        return true;
    }

    return false;
}

bool Window3::OnMouseMotion(MouseButton button, int x, int y, unsigned int)
{
    if (button == MOUSE_LEFT && mTrackball.GetActive())
    {
        mTrackball.SetFinalPoint(x, mYSize - 1 - y);
        mPVWMatrices.Update();
        return true;
    }

    return false;
}
