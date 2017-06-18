// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.2 (2016/11/14)

#include <GTEnginePCH.h>
#include <Applications/GLX/GteWindow.h>
#include <X11/Xlib.h>

namespace gte
{

Window::Parameters::Parameters()
    :
    display(nullptr),
    window(0),
    deviceCreationFlags(0)
{
}

Window::Parameters::Parameters(std::wstring const& inTitle,
    int inXOrigin, int inYOrigin, int inXSize, int inYSize)
    :
    WindowBase::Parameters(inTitle, inXOrigin, inYOrigin, inXSize, inYSize),
    display(nullptr),
    window(0),
    deviceCreationFlags(0)
{
}

Window::~Window()
{
}

Window::Window(Parameters& parameters)
    :
    WindowBase(parameters),
    mDisplay(parameters.display),
    mWindow(parameters.window),
    mButtonDown{false, false, false, false, false, false, false, false},
    mShiftDown(false),
    mControlDown(false),
    mAltDown(false),
    mCommandDown(false),
    mEngine(std::static_pointer_cast<GraphicsEngine>(mBaseEngine))
{
}

void Window::ShowWindow()
{
    XMapWindow(mDisplay, mWindow);
}

void Window::SetMousePosition(int x, int y)
{
    XWarpPointer(mDisplay, 0, mWindow, 0, 0, 0, 0, x, y);
    XFlush(mDisplay);
}

void Window::GetMousePosition(int& x, int& y) const
{
    XID rootWindow, childWindow;
    int rootX, rootY;
    unsigned int modifier;
    XQueryPointer(mDisplay, mWindow, &rootWindow, &childWindow,
        &rootX, &rootY, &x, &y, &modifier);
}

void Window::OnClose()
{
    XDestroyWindow(mDisplay, mWindow);
}

int Window::ProcessedEvent()
{
    if (!XPending(mDisplay))
    {
        return EVT_NONE_PENDING;
    }

    XEvent evt;
    XNextEvent(mDisplay, &evt);
    int index;
    bool state;

    if (evt.type == ButtonPress || evt.type == ButtonRelease)
    {
        OnMouseClick(evt.xbutton.button, evt.xbutton.type,
            evt.xbutton.x, evt.xbutton.y, evt.xbutton.state);

        mButtonDown[evt.xbutton.button] = (evt.type == ButtonPress);
        return EVT_PROCESSED;
    }

    if (evt.type == MotionNotify)
    {
        int button = MOUSE_NONE;
        for (int i = MOUSE_LEFT; i <= MOUSE_RIGHT; ++i)
        {
            if (mButtonDown[i])
            {
                button = i;
                break;
            }
        }
        OnMouseMotion(button, evt.xmotion.x, evt.xmotion.y, evt.xmotion.state);
        return EVT_PROCESSED;
    }

    if (evt.type == KeyPress || evt.type == KeyRelease)
    {
        int keysyms_per_keycode_return;
        KeySym* pKeySym = XGetKeyboardMapping(mDisplay,
            evt.xkey.keycode, 1, &keysyms_per_keycode_return);
        KeySym& keySym = *pKeySym;
        int key = (keySym & 0x00FF);

        // Quit application if the KEY_ESCAPE key is pressed.
        if (key == KEY_ESCAPE)
        {
            XFree(pKeySym);
            return EVT_QUIT;
        }

        // Adjust for special keys from the key pad or the number pad.
        if ((keySym & 0xFF00) != 0)
        {
            if (0x50 <= key && key <= 0x57)
            {
                // keypad Home, {L,U,R,D}Arrow, Pg{Up,Dn}, End
                key += 0x45;
            }
            else if (key == 0x63)
            {
                // keypad Insert
                key = 0x9e;
            }
            else if (key == 0xFF)
            {
                // keypad Delete
                key = 0x9f;
            }
            else if (key == 0xE1 || key == 0xE2)
            {
                // L-shift or R-shift
                key = KEY_SHIFT;
                mShiftDown = (evt.type == KeyPress);
            }
            else if (key == 0xE3 || key == 0xE4)
            {
                // L-ctrl or R-ctrl
                key = KEY_CONTROL;
                mControlDown = (evt.type == KeyPress);
            }
            else if (key == 0xE9 || key == 0xEA)
            {
                // L-alt or R-alt
                key = KEY_ALT;
                mAltDown = (evt.type == KeyPress);
            }
            else if (key == 0xEB || key == 0xEC)
            {
                key = KEY_COMMAND;
                mCommandDown = (evt.type == KeyPress);
            }
        }

        if ((KEY_HOME <= key &&  key <= KEY_END)
            || (KEY_F1 <= key   &&  key <= KEY_F12)
            || (KEY_SHIFT <= key && key <= KEY_COMMAND))
        {
            if (evt.type == KeyPress)
            {
                OnKeyDown(key, evt.xbutton.x, evt.xbutton.y);
            }
            else
            {
                OnKeyUp(key, evt.xbutton.x, evt.xbutton.y);
            }
        }
        else
        {
            if (evt.type == KeyPress)
            {
                // Get key-modifier state.  Adjust for shift state.
                unsigned char ucKey = static_cast<unsigned char>(key);
                if (mShiftDown && 'a' <= ucKey && ucKey <= 'z')
                {
                    ucKey = static_cast<unsigned char>(key - 32);
                }
                OnCharPress(ucKey, evt.xbutton.x, evt.xbutton.y);
            }
        }
        XFree(pKeySym);
        return EVT_PROCESSED;
    }

    if (evt.type == Expose)
    {
        OnDisplay();
        return EVT_PROCESSED;
    }

    if (evt.type == ConfigureNotify)
    {
        OnMove(evt.xconfigure.x, evt.xconfigure.y);
        OnResize(evt.xconfigure.width, evt.xconfigure.height);
        return EVT_PROCESSED;
    }

    if (evt.type == ClientMessage)
    {
        Atom* wmDelete = nullptr;
        int count;
        if (XGetWMProtocols(mDisplay, mWindow, &wmDelete, &count))
        {
            if (evt.xclient.data.l[0] == *wmDelete)
            {
                return EVT_QUIT;
            }
        }
    }

    return EVT_NONE_PENDING;
}

int const WindowBase::KEY_ESCAPE = 0x1B;
int const WindowBase::KEY_HOME = 0x95;
int const WindowBase::KEY_LEFT = 0x96;
int const WindowBase::KEY_UP = 0x97;
int const WindowBase::KEY_RIGHT = 0x98;
int const WindowBase::KEY_DOWN = 0x99;
int const WindowBase::KEY_PAGE_UP = 0x9A;
int const WindowBase::KEY_PAGE_DOWN = 0x9B;
int const WindowBase::KEY_END = 0x9C;
int const WindowBase::KEY_INSERT = 0x9E;
int const WindowBase::KEY_DELETE = 0x9F;
int const WindowBase::KEY_F1 = 0xBE;
int const WindowBase::KEY_F2 = 0xBF;
int const WindowBase::KEY_F3 = 0xC0;
int const WindowBase::KEY_F4 = 0xC1;
int const WindowBase::KEY_F5 = 0xC2;
int const WindowBase::KEY_F6 = 0xC3;
int const WindowBase::KEY_F7 = 0xC4;
int const WindowBase::KEY_F8 = 0xC5;
int const WindowBase::KEY_F9 = 0xC6;
int const WindowBase::KEY_F10 = 0xC7;
int const WindowBase::KEY_F11 = 0xC8;
int const WindowBase::KEY_F12 = 0xC9;
int const WindowBase::KEY_BACKSPACE = 0x08;
int const WindowBase::KEY_TAB = 0x09;
int const WindowBase::KEY_ENTER = 0x0D;
int const WindowBase::KEY_RETURN = 0x0D;

int const WindowBase::KEY_SHIFT = 0xE1;  // L-shift
int const WindowBase::KEY_CONTROL = 0xE3;  // L-ctrl
int const WindowBase::KEY_ALT = 0xE9;  // L-alt
int const WindowBase::KEY_COMMAND = 0xEB;  // L-command

int const WindowBase::MOUSE_NONE = 0x0000;
int const WindowBase::MOUSE_LEFT = 0x0001;
int const WindowBase::MOUSE_MIDDLE = 0x0002;
int const WindowBase::MOUSE_RIGHT = 0x0003;
int const WindowBase::MOUSE_DOWN = 0x0004;
int const WindowBase::MOUSE_UP = 0x0005;

int const WindowBase::MODIFIER_CONTROL = 0x0004;
int const WindowBase::MODIFIER_LBUTTON = 0x0001;
int const WindowBase::MODIFIER_MBUTTON = 0x0002;
int const WindowBase::MODIFIER_RBUTTON = 0x0003;
int const WindowBase::MODIFIER_SHIFT = 0x0001;

}
