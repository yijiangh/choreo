// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Applications/GteEnvironment.h>
#include <cstdlib>
#include <fstream>
using namespace gte;

Environment::~Environment()
{
}

Environment::Environment()
{
}

std::string Environment::GetVariable(std::string const& name)
{
#if defined(__MSWINDOWS__)
    size_t size;
    getenv_s(&size, nullptr, 0, name.c_str());
    if (size > 0)
    {
        std::vector<char> tmpvar(size);
        errno_t result = getenv_s(&size, tmpvar.data(), size, name.c_str());
        std::string var = (result == 0 ? std::string(tmpvar.data()) : "");
        return var;
    }
    else
    {
        return "";
    }
#endif

#if defined(__LINUX__)
    char const* variable = getenv(name.c_str());
    return variable ? std::string(variable) : std::string("");
#endif
}

int Environment::GetNumDirectories() const
{
    return static_cast<int>(mDirectories.size());
}

std::string Environment::Get(int i) const
{
    if (0 <= i && i < static_cast<int>(mDirectories.size()))
    {
        return mDirectories[i];
    }

    LogError("Invalid index.");
    return "";
}

bool Environment::Insert(std::string const& directory)
{
    if (directory.size() > 0)
    {
        for (auto& d : mDirectories)
        {
            if (directory == d
                || directory + "/" == d
                || directory + "\\" == d)
            {
                return false;
            }
        }

        // Ensure all directories are terminated with a slash.
        char lastChar = directory[directory.size() - 1];
        if (lastChar == '\\' || lastChar == '/')
        {
            mDirectories.push_back(directory);
        }
        else
        {
            mDirectories.push_back(directory + "/");
        }
        return true;
    }

    LogError("Insert expects non-empty inputs.");
    return false;
}

bool Environment::Remove(std::string const& directory)
{
    auto iter = mDirectories.begin(), end = mDirectories.end();
    for (/**/; iter != end; ++iter)
    {
        if (directory == *iter)
        {
            mDirectories.erase(iter);
            return true;
        }
    }
    return false;
}

void Environment::RemoveAll()
{
    mDirectories.clear();
}

std::string Environment::GetPath(std::string const& name) const
{
    for (auto const& directory : mDirectories)
    {
        std::string decorated = directory + name;
        std::ifstream input(decorated, std::ios::binary);
        if (input)
        {
            input.close();
            return decorated;
        }
    }
    return "";
}
