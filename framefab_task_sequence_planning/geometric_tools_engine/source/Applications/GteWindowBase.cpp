// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Applications/GteWindowBase.h>
using namespace gte;

WindowBase::Parameters::Parameters()
    :
    title(L""),
    xOrigin(0),
    yOrigin(0),
    xSize(0),
    ySize(0),
    allowResize(false),
    created(false)
{
}

WindowBase::Parameters::Parameters(std::wstring const& inTitle,
    int inXOrigin, int inYOrigin, int inXSize, int inYSize)
    :
    title(inTitle),
    xOrigin(inXOrigin),
    yOrigin(inYOrigin),
    xSize(inXSize),
    ySize(inYSize),
    allowResize(false),
    created(false)
{
}


WindowBase::~WindowBase()
{
}

WindowBase::WindowBase(Parameters& parameters)
    :
    mTitle(parameters.title),
    mXOrigin(parameters.xOrigin),
    mYOrigin(parameters.yOrigin),
    mXSize(parameters.xSize),
    mYSize(parameters.ySize),
    mAllowResize(parameters.allowResize),
    mIsMinimized(false),
    mIsMaximized(false),
    mBaseEngine(parameters.engine),
    mProgramFactory(parameters.factory)
{
}

void WindowBase::SetTitle(std::wstring const& title)
{
    mTitle = title;
}

void WindowBase::OnMove(int x, int y)
{
    mXOrigin = x;
    mYOrigin = y;
}

bool WindowBase::OnResize(int xSize, int ySize)
{
    mIsMinimized = false;
    mIsMaximized = false;

    if (xSize != mXSize || ySize != mYSize)
    {
        mXSize = xSize;
        mYSize = ySize;

        if (mBaseEngine)
        {
            mBaseEngine->Resize(xSize, ySize);
        }
        return true;
    }

    return false;
}

void WindowBase::OnMinimize()
{
    mIsMinimized = true;
    mIsMaximized = false;
}

void WindowBase::OnMaximize()
{
    mIsMinimized = false;
    mIsMaximized = true;
}

void WindowBase::OnDisplay()
{
    // Stub for derived classes.
}

void WindowBase::OnIdle()
{
    // Stub for derived classes.
}

bool WindowBase::OnCharPress(unsigned char key, int, int)
{
    if (key == KEY_ESCAPE)
    {
        // Quit the application when the 'escape' key is pressed.
        OnClose();
        return true;
    }

    if (key == ' ')
    {
        mTimer.Reset();
        return true;
    }

    return false;
}

bool WindowBase::OnKeyDown(int, int, int)
{
    // Stub for derived classes.
    return false;
}

bool WindowBase::OnKeyUp(int, int, int)
{
    // Stub for derived classes.
    return false;
}

bool WindowBase::OnMouseClick(int, int, int, int, unsigned int)
{
    // stub for derived classes
    return false;
}

bool WindowBase::OnMouseMotion(int, int, int, unsigned int)
{
    // stub for derived classes
    return false;
}

bool WindowBase::OnMouseWheel(int, int, int, unsigned int)
{
    // Stub for derived classes.
    return false;
}

void WindowBase::SetMousePosition(int, int)
{
    // Stub for derived classes.
}

void WindowBase::GetMousePosition(int&, int&) const
{
    // Stub for derived classes.
}

void WindowBase::OnClose()
{
    // Stub for derived classes.
}

std::string WindowBase::GetGTEPath()
{
    std::string path = mEnvironment.GetVariable("GTE_PATH");
    if (path == "")
    {
        LogError("You must create the environment variable GTE_PATH.");
        return "";
    }
    return path;
}
