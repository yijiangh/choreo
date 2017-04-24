// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteProgramDefines.h>
using namespace gte;


ProgramDefines::ProgramDefines()
{
}

void ProgramDefines::Update(std::string const& name, std::string const& value)
{
    // If an item already exists with the specified name, update it.
    for (auto& definition : mDefinitions)
    {
        if (name == definition.first)
        {
            definition.second = value;
            return;
        }
    }

    // The item is new, so append it.
    mDefinitions.push_back(std::make_pair(name, value));
}

void ProgramDefines::Remove(std::string const& name)
{
    for (auto iter = mDefinitions.begin(); iter != mDefinitions.end(); ++iter)
    {
        if (name == iter->first)
        {
            mDefinitions.erase(iter);
            break;
        }
    }
}

void ProgramDefines::Clear()
{
    mDefinitions.clear();
}

