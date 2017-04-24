// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Applications/MSW/GteMSWWindow.h>
using namespace gte;

MSWWindow::Parameters::Parameters()
    :
    handle(nullptr),
    hscrollBar(false),
    vscrollBar(false)
{
}

MSWWindow::Parameters::Parameters(std::wstring const& inTitle,
    int inXOrigin, int inYOrigin, int inXSize, int inYSize)
    :
    WindowBase::Parameters(inTitle, inXOrigin, inYOrigin, inXSize, inYSize),
    handle(nullptr),
    hscrollBar(false),
    vscrollBar(false)
{
}

MSWWindow::~MSWWindow()
{
}

MSWWindow::MSWWindow(Parameters& parameters)
    :
    WindowBase(parameters),
    mHandle(parameters.handle)
{
    // If scroll bars are requested, the MSWWindow-derived class constructor
    // should call the function MSWWindow::SetScrollInterval(...) to set the
    // scroll bar range.  Increments and decrements may be set separately
    // directly via class members.
    mHasScroll[0] = parameters.hscrollBar;
    mHasScroll[1] = parameters.vscrollBar;
    for (int i = 0; i < 2; ++i)
    {
        ZeroMemory(&mScrollInfo[i], sizeof(SCROLLINFO));
        mScrollInfo[i].cbSize = sizeof(SCROLLINFO);
        mScrollInfo[i].fMask = SIF_ALL;
        GetScrollInfo(mHandle, i, &mScrollInfo[i]);
        mScrollLoResDelta[i] = 1;
        mScrollHiResDelta[i] = 1;
    }
}

void MSWWindow::SetTitle(std::wstring const& title)
{
    WindowBase::SetTitle(title);
    SetWindowText(mHandle, title.c_str());
}

void MSWWindow::SetMousePosition(int x, int y)
{
    POINT point = { static_cast<LONG>(x), static_cast<LONG>(y) };
    ClientToScreen(mHandle, &point);
    SetCursorPos(point.x, point.y);
}

void MSWWindow::GetMousePosition(int& x, int& y) const
{
    POINT point;
    GetCursorPos(&point);
    ScreenToClient(mHandle, &point);
    x = static_cast<int>(point.x);
    y = static_cast<int>(point.y);
}

void MSWWindow::OnClose()
{
    PostQuitMessage(0);
}

void MSWWindow::SetScrollInterval(int bar, int minValue, int maxValue)
{
    mScrollInfo[bar].fMask = SIF_RANGE;
    mScrollInfo[bar].nMin = minValue;
    mScrollInfo[bar].nMax = maxValue;
    SetScrollInfo(mHandle, bar, &mScrollInfo[bar], FALSE);
}

void MSWWindow::GetScrollInterval(int bar, int& minValue, int& maxValue) const
{
    mScrollInfo[bar].fMask = SIF_RANGE;
    GetScrollInfo(mHandle, bar, &mScrollInfo[bar]);
    minValue = mScrollInfo[bar].nMin;
    maxValue = mScrollInfo[bar].nMax;
}

int MSWWindow::SetScrollPosition(int bar, int value)
{
    mScrollInfo[bar].fMask = SIF_POS;
    mScrollInfo[bar].nPos = value;
    return SetScrollInfo(mHandle, bar, &mScrollInfo[bar], FALSE);
}

int MSWWindow::GetScrollPosition(int bar) const
{
    mScrollInfo[bar].fMask = SIF_POS;
    GetScrollInfo(mHandle, bar, &mScrollInfo[bar]);
    return mScrollInfo[bar].nPos;
}

int MSWWindow::OnScrollIncrementLoRes(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_POS;
    GetScrollInfo(mHandle, bar, &info);
    int delta = info.nMax - info.nPos;
    if (delta > 0)
    {
        delta = std::min(delta, mScrollLoResDelta[bar]);
        info.nPos += delta;
        SetScrollInfo(mHandle, bar, &info, TRUE);
    }
    return delta;
}

int MSWWindow::OnScrollDecrementLoRes(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_POS;
    GetScrollInfo(mHandle, bar, &info);
    int delta = info.nPos - info.nMin;
    if (delta > 0)
    {
        delta = std::min(delta, mScrollLoResDelta[bar]);
        info.nPos -= delta;
        SetScrollInfo(mHandle, bar, &info, TRUE);
    }
    return delta;
}

int MSWWindow::OnScrollIncrementHiRes(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_POS;
    GetScrollInfo(mHandle, bar, &info);
    int delta = info.nMax - info.nPos;
    if (delta > 0)
    {
        delta = std::min(delta, mScrollHiResDelta[bar]);
        info.nPos += delta;
        SetScrollInfo(mHandle, bar, &info, TRUE);
    }
    return delta;
}

int MSWWindow::OnScrollDecrementHiRes(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_POS;
    GetScrollInfo(mHandle, bar, &info);
    int delta = info.nPos - info.nMin;
    if (delta > 0)
    {
        delta = std::min(delta, mScrollHiResDelta[bar]);
        info.nPos -= delta;
        SetScrollInfo(mHandle, bar, &info, TRUE);
    }
    return delta;
}

int MSWWindow::OnScrollTracking(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_TRACKPOS;
    GetScrollInfo(mHandle, bar, &info);
    return info.nTrackPos;
}

int MSWWindow::OnScrollEndTracking(int bar)
{
    SCROLLINFO& info = mScrollInfo[bar];
    info.fMask = SIF_TRACKPOS;
    GetScrollInfo(mHandle, bar, &info);
    info.fMask = SIF_POS;
    info.nPos = info.nTrackPos;
    SetScrollInfo(mHandle, bar, &info, TRUE);
    return info.nTrackPos;
}

// Named class-static constants are shared between DX11 and WGL.
int const WindowBase::KEY_ESCAPE = VK_ESCAPE;
int const WindowBase::KEY_LEFT = VK_LEFT;
int const WindowBase::KEY_RIGHT = VK_RIGHT;
int const WindowBase::KEY_UP = VK_UP;
int const WindowBase::KEY_DOWN = VK_DOWN;
int const WindowBase::KEY_HOME = VK_HOME;
int const WindowBase::KEY_END = VK_END;
int const WindowBase::KEY_PAGE_UP = VK_PRIOR;
int const WindowBase::KEY_PAGE_DOWN = VK_NEXT;
int const WindowBase::KEY_INSERT = VK_INSERT;
int const WindowBase::KEY_DELETE = VK_DELETE;
int const WindowBase::KEY_F1 = VK_F1;
int const WindowBase::KEY_F2 = VK_F2;
int const WindowBase::KEY_F3 = VK_F3;
int const WindowBase::KEY_F4 = VK_F4;
int const WindowBase::KEY_F5 = VK_F5;
int const WindowBase::KEY_F6 = VK_F6;
int const WindowBase::KEY_F7 = VK_F7;
int const WindowBase::KEY_F8 = VK_F8;
int const WindowBase::KEY_F9 = VK_F9;
int const WindowBase::KEY_F10 = VK_F10;
int const WindowBase::KEY_F11 = VK_F11;
int const WindowBase::KEY_F12 = VK_F12;
int const WindowBase::KEY_BACKSPACE = VK_BACK;
int const WindowBase::KEY_TAB = VK_TAB;
int const WindowBase::KEY_ENTER = VK_RETURN;
int const WindowBase::KEY_RETURN = VK_RETURN;

int const WindowBase::KEY_SHIFT = VK_SHIFT;
int const WindowBase::KEY_CONTROL = VK_CONTROL;
int const WindowBase::KEY_ALT = 0;      // not currently handled
int const WindowBase::KEY_COMMAND = 0;  // not currently handled

int const WindowBase::MOUSE_NONE = 0;
int const WindowBase::MOUSE_LEFT = 1;
int const WindowBase::MOUSE_MIDDLE = 2;
int const WindowBase::MOUSE_RIGHT= 3;
int const WindowBase::MOUSE_UP = 0;
int const WindowBase::MOUSE_DOWN = 1;

int const WindowBase::MODIFIER_CONTROL = MK_CONTROL;
int const WindowBase::MODIFIER_LBUTTON = MK_LBUTTON;
int const WindowBase::MODIFIER_MBUTTON = MK_MBUTTON;
int const WindowBase::MODIFIER_RBUTTON = MK_RBUTTON;
int const WindowBase::MODIFIER_SHIFT = MK_SHIFT;
