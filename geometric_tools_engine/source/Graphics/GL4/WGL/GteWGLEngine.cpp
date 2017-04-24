// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/06/30)

#include <GTEnginePCH.h>
#include <Graphics/GL4/WGL/GteWGLEngine.h>
using namespace gte;

WGLEngine::~WGLEngine()
{
    Terminate();
}

WGLEngine::WGLEngine(HWND handle, bool saveDriverInfo, int requiredMajor, int requiredMinor)
    :
    GL4Engine(),
    mHandle(handle),
    mDevice(nullptr),
    mImmediate(nullptr),
    mComputeWindowAtom(0)
{
    Initialize(requiredMajor, requiredMinor, saveDriverInfo);
}

WGLEngine::WGLEngine(bool saveDriverInfo, int requiredMajor, int requiredMinor)
    :
    GL4Engine(),
    mHandle(nullptr),
    mDevice(nullptr),
    mImmediate(nullptr),
    mComputeWindowAtom(0)
{
    mComputeWindowClass = L"GL4ComputeWindowClass" +
        std::to_wstring(reinterpret_cast<uint64_t>(this));

    WNDCLASS wc;
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = DefWindowProc;
    wc.cbClsExtra = 0;
    wc.cbWndExtra = 0;
    wc.hInstance = nullptr;
    wc.hIcon = LoadIcon(0, IDI_APPLICATION);
    wc.hCursor = LoadCursor(0, IDC_ARROW);
    wc.hbrBackground = static_cast<HBRUSH>(GetStockObject(WHITE_BRUSH));
    wc.lpszClassName = mComputeWindowClass.c_str();
    wc.lpszMenuName = nullptr;
    mComputeWindowAtom = RegisterClass(&wc);

    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    RECT rect = { 0, 0, 15, 15 };
    if (AdjustWindowRect(&rect, style, FALSE) != FALSE)
    {
        int xSize = static_cast<int>(rect.right - rect.left + 1);
        int ySize = static_cast<int>(rect.bottom - rect.top + 1);
        mHandle = CreateWindow(mComputeWindowClass.c_str(), L"", style,
            0, 0, xSize, ySize, nullptr, nullptr, nullptr, nullptr);
        Initialize(requiredMajor, requiredMinor, saveDriverInfo);
    }
    else
    {
        UnregisterClass(mComputeWindowClass.c_str(), 0);
        mComputeWindowAtom = 0;
    }
}

bool WGLEngine::IsActive() const
{
    return mImmediate == wglGetCurrentContext();
}

void WGLEngine::MakeActive()
{
    if (mImmediate != wglGetCurrentContext())
    {
        wglMakeCurrent(mDevice, mImmediate);
    }
}

void WGLEngine::DisplayColorBuffer(unsigned int syncInterval)
{
    wglSwapIntervalEXT(syncInterval > 0 ? 1 : 0);
    SwapBuffers(mDevice);
}

bool WGLEngine::Initialize(int requiredMajor, int requiredMinor, bool saveDriverInfo)
{
    if (!mHandle)
    {
        LogError("Invalid window handle.");
        return false;
    }

    mDevice = GetDC(mHandle);
    if (!mDevice)
    {
        LogError("Invalid device context.");
        mHandle = nullptr;
        return false;
    }

    RECT rect;
    BOOL result = GetClientRect(mHandle, &rect); (void)result;
    mXSize = static_cast<unsigned int>(rect.right - rect.left);
    mYSize = static_cast<unsigned int>(rect.bottom - rect.top);

    // Select the format for the drawing surface.
    PIXELFORMATDESCRIPTOR pfd;
    memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
    pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion = 1;
    pfd.dwFlags =
        PFD_DRAW_TO_WINDOW |
        PFD_SUPPORT_OPENGL |
        PFD_GENERIC_ACCELERATED |
        PFD_DOUBLEBUFFER;

    // Create an R8G8B8A8 buffer.
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;

    // Create a D24S8 depth-stencil buffer.
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;

    // Set the pixel format for the rendering context.
    int pixelFormat = ChoosePixelFormat(mDevice, &pfd);
    if (pixelFormat == 0)
    {
        LogError("ChoosePixelFormat failed.");
        ReleaseDC(mHandle, mDevice);
        mXSize = 0;
        mYSize = 0;
        mDevice = nullptr;
        mHandle = nullptr;
        return false;
    }

    if (!SetPixelFormat(mDevice, pixelFormat, &pfd))
    {
        LogError("SetPixelFormat failed.");
        ReleaseDC(mHandle, mDevice);
        mXSize = 0;
        mYSize = 0;
        mDevice = nullptr;
        mHandle = nullptr;
        return false;
    }

    // Create an OpenGL context.
    mImmediate = wglCreateContext(mDevice);
    if (!mImmediate)
    {
        LogError("wglCreateContext failed.");
        ReleaseDC(mHandle, mDevice);
        mXSize = 0;
        mYSize = 0;
        mDevice = nullptr;
        mHandle = nullptr;
        return false;
    }

    // Activate the context.
    if (!wglMakeCurrent(mDevice, mImmediate))
    {
        LogError("wglMakeCurrent failed.");
        wglDeleteContext(mImmediate);
        ReleaseDC(mHandle, mDevice);
        mXSize = 0;
        mYSize = 0;
        mImmediate = nullptr;
        mDevice = nullptr;
        mHandle = nullptr;
        return false;
    }

    // Get the function pointers for WGL.
    InitializeWGL();

    // Get the function pointers for OpenGL; initialize the viewport,
    // default global state, and default font.
    return GL4Engine::Initialize(requiredMajor, requiredMinor, saveDriverInfo);
}

void WGLEngine::Terminate()
{
    GL4Engine::Terminate();

    if (mHandle && mDevice && mImmediate)
    {
        wglMakeCurrent(mDevice, nullptr);
        wglDeleteContext(mImmediate);
        ReleaseDC(mHandle, mDevice);
    }

    if (mComputeWindowAtom)
    {
        DestroyWindow(mHandle);
        UnregisterClass(mComputeWindowClass.c_str(), 0);
    }
}
