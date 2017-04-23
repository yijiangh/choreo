// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.2 (2016/11/12)

#pragma once

#include <Applications/MSW/GteMSWWindow.h>
#include <map>
#include <memory>

namespace gte
{

class GTE_IMPEXP MSWWindowSystem
{
public:
    // Abstract base class.  The derived classes are instantiable and
    // singletons.
    virtual ~MSWWindowSystem();
protected:
    MSWWindowSystem();

public:
    // Create and destroy windows.  Derived classes may extend the inputs
    // using a nested class derived from Window::Parameters
    template <typename WindowType>
    std::shared_ptr<WindowType> Create(typename WindowType::Parameters& parameters);

    template <typename WindowType>
    void Destroy(std::shared_ptr<WindowType>& window);

    enum
    {
        DEFAULT_ACTION = 0,
        NO_IDLE_LOOP = 1
    };

    template <typename WindowType>
    void MessagePump(std::shared_ptr<WindowType> const& window, unsigned int flags);

protected:
    // Get the true window size for the specified client size.  The true size
    // includes extra space for window decorations (window border, menu bar,
    // and so on).  This information is useful to know before creating a
    // window to ensure the to-be-created window fits within the monitor
    // resolution.
    static bool GetWindowRectangle(int xClientSize, int yClientSize,
        DWORD style, RECT& windowRectangle);

    // Window creation and destruction.
    void CreateFrom(MSWWindow::Parameters& parameters);
    virtual void CreateEngineAndProgramFactory(MSWWindow::Parameters& parameters) = 0;

    // Extraction of cursor location, avoiding the extraction in <windows.h>
    // that does not work when you have dual monitors.
    static void Extract(LPARAM lParam, int& x, int& y);
    static void Extract(WPARAM wParam, int& x, int& y);

    // The event handler.
    static LRESULT CALLBACK WindowProcedure(HWND handle, UINT message, WPARAM wParam, LPARAM lParam);

    wchar_t const* mWindowClassName;
    ATOM mAtom;
    std::map<HWND, std::shared_ptr<MSWWindow>> mHandleMap;
};


template <typename WindowType>
std::shared_ptr<WindowType> MSWWindowSystem::Create(typename WindowType::Parameters& parameters)
{
    CreateFrom(parameters);
    if (parameters.created)
    {
        std::shared_ptr<WindowType> window = std::make_shared<WindowType>(parameters);
        mHandleMap[parameters.handle] = window;
        if (parameters.created)
        {
            return window;
        }
        Destroy(window);
    }
    // else: CreateFrom will report the problem via the logger system.
    return nullptr;
}

template <typename WindowType>
void MSWWindowSystem::Destroy(std::shared_ptr<WindowType>& window)
{
    if (window)
    {
        HWND handle = window->GetHandle();
        mHandleMap.erase(handle);
        window = nullptr;
        DestroyWindow(handle);
    }
}

template <typename WindowType>
void MSWWindowSystem::MessagePump(std::shared_ptr<WindowType> const& window, unsigned int flags)
{
    if (window)
    {
        HWND handle = window->GetHandle();
        ShowWindow(handle, SW_SHOW);
        UpdateWindow(handle);

        for (;;)
        {
            if (flags & NO_IDLE_LOOP)
            {
                WaitMessage();
            }

            MSG msg;
            if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
            {
                if (msg.message == WM_QUIT)
                {
                    break;
                }

                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            else
            {
                if (!(flags & NO_IDLE_LOOP))
                {
                    if (!window->IsMinimized())
                    {
                        window->OnIdle();
                    }
                }
            }
        }
    }
}

}
