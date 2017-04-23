// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Graphics/GteProgramFactory.h>
#include <fstream>
using namespace gte;


ProgramFactory::~ProgramFactory()
{
}

ProgramFactory::ProgramFactory()
    :
    version(""),
    vsEntry(""),
    psEntry(""),
    gsEntry(""),
    csEntry(""),
    defines(),
    flags(0)
{
}

std::shared_ptr<VisualProgram> ProgramFactory::CreateFromFiles(
    std::string const& vsFile, std::string const& psFile,
    std::string const& gsFile)
{
    if (vsFile == "" || psFile == "")
    {
        LogError("A program must have a vertex shader and a pixel shader.");
        return nullptr;
    }

    std::string vsSource = GetStringFromFile(vsFile);
    if (vsSource == "")
    {
        return nullptr;
    }

    std::string psSource = GetStringFromFile(psFile);
    if (psSource == "")
    {
        return nullptr;
    }

    std::string gsSource = "";
    if (gsFile != "")
    {
        gsSource = GetStringFromFile(gsFile);
        if (gsSource == "")
        {
            return nullptr;
        }
    }

    return CreateFromNamedSources(vsFile, vsSource, psFile, psSource, gsFile,
        gsSource);
}

std::shared_ptr<VisualProgram> ProgramFactory::CreateFromSources(
    std::string const& vsSource, std::string const& psSource,
    std::string const& gsSource)
{
    return CreateFromNamedSources("vs", vsSource, "ps", psSource, "gs",
        gsSource);
}

std::shared_ptr<ComputeProgram> ProgramFactory::CreateFromFile(
    std::string const& csFile)
{
    if (csFile == "")
    {
        LogError("A program must have a compute shader.");
        return nullptr;
    }

    std::string csSource = GetStringFromFile(csFile);
    if (csSource == "")
    {
        return nullptr;
    }

    return CreateFromNamedSource(csFile, csSource);
}

std::shared_ptr<ComputeProgram> ProgramFactory::CreateFromSource(
    std::string const& csSource)
{
    return CreateFromNamedSource("cs", csSource);
}

std::string ProgramFactory::GetStringFromFile(std::string const& filename)
{
    std::string source = "";
    std::ifstream input(filename);
    if (input)
    {
        while (!input.eof())
        {
            std::string line;
            getline(input, line);
            source += line + "\n";
        }
        input.close();
    }
    else
    {
        LogError("Cannot open file " + filename);
    }
    return source;
}

void ProgramFactory::PushDefines()
{
    mDefinesStack.push(defines);
    defines.Clear();
}

void ProgramFactory::PopDefines()
{
    if (mDefinesStack.size() > 0)
    {
        defines = mDefinesStack.top();
        mDefinesStack.pop();
    }
}

void ProgramFactory::PushFlags()
{
    mFlagsStack.push(flags);
    flags = 0;
}

void ProgramFactory::PopFlags()
{
    if (mFlagsStack.size() > 0)
    {
        flags = mFlagsStack.top();
        mFlagsStack.pop();
    }
}

