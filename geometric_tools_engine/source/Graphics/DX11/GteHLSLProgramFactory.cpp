// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2016/09/12)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#if defined(GTE_USE_DX12)
#include <Graphics/DX12/GteDX12Include.h>
#else
#include <Graphics/DX11/GteDX11Include.h>
#endif
#include <Graphics/DX11/GteHLSLComputeProgram.h>
#include <Graphics/DX11/GteHLSLProgramFactory.h>
#include <Graphics/DX11/GteHLSLShaderFactory.h>
#include <Graphics/DX11/GteHLSLVisualProgram.h>
using namespace gte;

std::string HLSLProgramFactory::defaultVersion = "5_0";
std::string HLSLProgramFactory::defaultVSEntry = "VSMain";
std::string HLSLProgramFactory::defaultPSEntry = "PSMain";
std::string HLSLProgramFactory::defaultGSEntry = "GSMain";
std::string HLSLProgramFactory::defaultCSEntry = "CSMain";
unsigned int HLSLProgramFactory::defaultFlags = (
    D3DCOMPILE_ENABLE_STRICTNESS |
    D3DCOMPILE_IEEE_STRICTNESS |
    D3DCOMPILE_OPTIMIZATION_LEVEL3);

HLSLProgramFactory::~HLSLProgramFactory()
{
}

HLSLProgramFactory::HLSLProgramFactory()
{
    version = defaultVersion;
    vsEntry = defaultVSEntry;
    psEntry = defaultPSEntry;
    gsEntry = defaultGSEntry;
    csEntry = defaultCSEntry;
    flags = defaultFlags;
}

int HLSLProgramFactory::GetAPI() const
{
    return PF_HLSL;
}

std::shared_ptr<VisualProgram> HLSLProgramFactory::CreateFromBytecode(
    std::vector<unsigned char> const& vsBytecode,
    std::vector<unsigned char> const& psBytecode,
    std::vector<unsigned char> const& gsBytecode)
{
    if (vsBytecode.size() == 0 || psBytecode.size() == 0)
    {
        LogError("A program must have a vertex shader and a pixel shader.");
        return nullptr;
    }

    std::shared_ptr<VertexShader> vshader;
    std::shared_ptr<PixelShader> pshader;
    std::shared_ptr<GeometryShader> gshader;

    HLSLShader hlslVShader = HLSLShaderFactory::CreateFromBytecode("vs",
        vsEntry, std::string("vs_") + version, vsBytecode.size(),
        vsBytecode.data());
    if (hlslVShader.IsValid())
    {
        vshader =
            std::static_pointer_cast<VertexShader>(std::make_shared<Shader>(hlslVShader));
    }
    else
    {
        return nullptr;
    }

    HLSLShader hlslPShader = HLSLShaderFactory::CreateFromBytecode("ps",
        psEntry, std::string("ps_") + version, psBytecode.size(),
        psBytecode.data());
    if (hlslPShader.IsValid())
    {
        pshader =
            std::static_pointer_cast<PixelShader>(std::make_shared<Shader>(hlslPShader));
    }
    else
    {
        return nullptr;
    }

    HLSLShader hlslGShader;
    if (gsBytecode.size() > 0)
    {
        hlslGShader = HLSLShaderFactory::CreateFromBytecode("gs",
            gsEntry, std::string("gs_") + version, gsBytecode.size(),
            gsBytecode.data());
        if (hlslGShader.IsValid())
        {
            gshader =
                std::static_pointer_cast<GeometryShader>(std::make_shared<Shader>(hlslGShader));
        }
        else
        {
            return nullptr;
        }
    }

    std::shared_ptr<HLSLVisualProgram> program = std::make_shared<HLSLVisualProgram>();
    program->SetVShader(vshader);
    program->SetPShader(pshader);
    program->SetGShader(gshader);
    return program;
}

std::shared_ptr<ComputeProgram> HLSLProgramFactory::CreateFromNamedSource(
    std::string const& csName, std::string const& csSource)
{
    if (csSource == "")
    {
        LogError("A program must have a compute shader.");
        return nullptr;
    }

    HLSLShader hlslCShader = HLSLShaderFactory::CreateFromString(csName,
        csSource, csEntry, std::string("cs_") + version, defines, flags);
    if (hlslCShader.IsValid())
    {
        std::shared_ptr<ComputeShader> cshader =
            std::static_pointer_cast<ComputeShader>(std::make_shared<Shader>(hlslCShader));

        std::shared_ptr<HLSLComputeProgram> program = std::make_shared<HLSLComputeProgram>();
        program->SetCShader(cshader);
        return program;
    }
    else
    {
        return nullptr;
    }
}

std::shared_ptr<VisualProgram> HLSLProgramFactory::CreateFromNamedSources(
    std::string const& vsName, std::string const& vsSource,
    std::string const& psName, std::string const& psSource,
    std::string const& gsName, std::string const& gsSource)
{
    if (vsSource == "" || psSource == "")
    {
        LogError("A program must have a vertex shader and a pixel shader.");
        return nullptr;
    }

    std::shared_ptr<VertexShader> vshader;
    std::shared_ptr<PixelShader> pshader;
    std::shared_ptr<GeometryShader> gshader;

    HLSLShader hlslVShader = HLSLShaderFactory::CreateFromString(vsName,
        vsSource, vsEntry, std::string("vs_") + version, defines, flags);
    if (hlslVShader.IsValid())
    {
        vshader =
            std::static_pointer_cast<VertexShader>(std::make_shared<Shader>(hlslVShader));
    }
    else
    {
        return nullptr;
    }

    HLSLShader hlslPShader = HLSLShaderFactory::CreateFromString(psName,
        psSource, psEntry, std::string("ps_") + version, defines, flags);
    if (hlslPShader.IsValid())
    {
        pshader =
            std::static_pointer_cast<PixelShader>(std::make_shared<Shader>(hlslPShader));
    }
    else
    {
        return nullptr;
    }

    HLSLShader hlslGShader;
    if (gsSource != "")
    {
        hlslGShader = HLSLShaderFactory::CreateFromString(gsName,
            gsSource, gsEntry, std::string("gs_") + version, defines, flags);
        if (hlslGShader.IsValid())
        {
            gshader =
                std::static_pointer_cast<GeometryShader>(std::make_shared<Shader>(hlslGShader));
        }
        else
        {
            return nullptr;
        }
    }

    std::shared_ptr<HLSLVisualProgram> program = std::make_shared<HLSLVisualProgram>();
    program->SetVShader(vshader);
    program->SetPShader(pshader);
    program->SetGShader(gshader);
    return program;
}

std::shared_ptr<ComputeProgram> HLSLProgramFactory::CreateFromBytecode(
    std::vector<unsigned char> const& csBytecode)
{
    if (csBytecode.size() == 0)
    {
        LogError("A program must have a compute shader.");
        return nullptr;
    }

    HLSLShader hlslCShader = HLSLShaderFactory::CreateFromBytecode("cs",
        csEntry, std::string("cs_") + version, csBytecode.size(),
        csBytecode.data());
    if (hlslCShader.IsValid())
    {
        std::shared_ptr<ComputeShader> cshader =
            std::static_pointer_cast<ComputeShader>(std::make_shared<Shader>(hlslCShader));

        std::shared_ptr<HLSLComputeProgram> program = std::make_shared<HLSLComputeProgram>();
        program->SetCShader(cshader);
        return program;
    }
    else
    {
        return nullptr;
    }
}
