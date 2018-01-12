// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.2 (2016/12/09)

#include <GTEnginePCH.h>
#include <Graphics/GteFontArialW400H18.h>
#include <Graphics/GL4/GteGL4AtomicCounterBuffer.h>
#include <Graphics/GL4/GteGL4Engine.h>
#include <Graphics/GL4/GteGL4BlendState.h>
#include <Graphics/GL4/GteGL4ConstantBuffer.h>
#include <Graphics/GL4/GteGL4DepthStencilState.h>
#include <Graphics/GL4/GteGL4DrawTarget.h>
#include <Graphics/GL4/GteGL4IndexBuffer.h>
#include <Graphics/GL4/GteGL4RasterizerState.h>
#include <Graphics/GL4/GteGL4SamplerState.h>
#include <Graphics/GL4/GteGL4StructuredBuffer.h>
#include <Graphics/GL4/GteGL4Texture1.h>
#include <Graphics/GL4/GteGL4Texture1Array.h>
#include <Graphics/GL4/GteGL4Texture2.h>
#include <Graphics/GL4/GteGL4Texture2Array.h>
#include <Graphics/GL4/GteGL4Texture3.h>
#include <Graphics/GL4/GteGL4TextureCube.h>
#include <Graphics/GL4/GteGL4TextureCubeArray.h>
#include <Graphics/GL4/GteGL4VertexBuffer.h>
#include <Graphics/GL4/GteGLSLProgramFactory.h>
#include <Graphics/GL4/GteGLSLComputeProgram.h>
#include <Graphics/GL4/GteGLSLVisualProgram.h>
#if defined(NDEBUG)
#include <iostream>
#endif
using namespace gte;

//----------------------------------------------------------------------------
// Interface specific to GL4.
//----------------------------------------------------------------------------
GL4Engine::~GL4Engine()
{
}

GL4Engine::GL4Engine()
    :
    mMajor(0),
    mMinor(0),
    mMeetsRequirements(false)
{
    // Initialization of GraphicsEngine members that depend on GL4.
    mILMap = std::make_unique<GL4InputLayoutManager>();

    mCreateGEObject =
    {
        nullptr, // GT_GRAPHICS_OBJECT (abstract base)
        nullptr, // GT_RESOURCE (abstract base)
        nullptr, // GT_BUFFER (abstract base)
        &GL4ConstantBuffer::Create,
        nullptr, // &DX11TextureBuffer::Create,
        &GL4VertexBuffer::Create,
        &GL4IndexBuffer::Create,
        &GL4StructuredBuffer::Create,
        nullptr, // TODO:  Implement TypedBuffer
        nullptr, // &DX11RawBuffer::Create,
        nullptr, // &DX11IndirectArgumentsBuffer::Create,
        nullptr, // GT_TEXTURE (abstract base)
        nullptr, // GT_TEXTURE_SINGLE (abstract base)
        &GL4Texture1::Create,
        &GL4Texture2::Create,
        &GL4TextureRT::Create,
        &GL4TextureDS::Create,
        &GL4Texture3::Create,
        nullptr, // GT_TEXTURE_ARRAY (abstract base)
        &GL4Texture1Array::Create,
        &GL4Texture2Array::Create,
        &GL4TextureCube::Create,
        &GL4TextureCubeArray::Create,
        nullptr, // GT_SHADER (abstract base)
        nullptr, // &DX11VertexShader::Create,
        nullptr, // &DX11GeometryShader::Create,
        nullptr, // &DX11PixelShader::Create,
        nullptr, // &DX11ComputeShader::Create,
        nullptr, // GT_DRAWING_STATE (abstract base)
        &GL4SamplerState::Create,
        &GL4BlendState::Create,
        &GL4DepthStencilState::Create,
        &GL4RasterizerState::Create
    };

    mCreateGEDrawTarget = &GL4DrawTarget::Create;
}

void GL4Engine::CreateDefaultFont()
{
    std::shared_ptr<GLSLProgramFactory> factory = std::make_shared<GLSLProgramFactory>();
    mDefaultFont = std::make_shared<FontArialW400H18>(factory, 256);
    SetDefaultFont();
}

void GL4Engine::DestroyDefaultFont()
{
    if (mDefaultFont)
    {
        mDefaultFont = nullptr;
        mActiveFont = nullptr;
    }
}

bool GL4Engine::Initialize(int requiredMajor, int requiredMinor, bool saveDriverInfo)
{
    if (saveDriverInfo)
    {
        InitializeOpenGL(mMajor, mMinor, "OpenGLDriverInfo.txt");
    }
    else
    {
        InitializeOpenGL(mMajor, mMinor, nullptr);
    }

    mMeetsRequirements = (mMajor > requiredMajor ||
        (mMajor == requiredMajor && mMinor >= requiredMinor));

    if (mMeetsRequirements)
    {
        SetViewport(0, 0, mXSize, mYSize);
        SetDepthRange(0.0f, 1.0f);
        CreateDefaultGlobalState();
        CreateDefaultFont();
    }
#if defined(NDEBUG)
    else
    {
        std::string message = "OpenGL " + std::to_string(requiredMajor) + "."
            + std::to_string(requiredMinor) + " is required.";
        std::cout << message << std::endl;
    }
#endif
    return mMeetsRequirements;
}

void GL4Engine::Terminate()
{
    // The render state objects (and fonts) are destroyed first so that the
    // render state objects are removed from the bridges before they are
    // cleared later in the destructor.
    DestroyDefaultFont();
    DestroyDefaultGlobalState();

    // Need to remove all the RawBuffer objects used to manage atomic
    // counter buffers.
    mAtomicCounterRawBuffers.clear();

    GraphicsObject::UnsubscribeForDestruction(mGOListener);
    mGOListener = nullptr;

    DrawTarget::UnsubscribeForDestruction(mDTListener);
    mDTListener = nullptr;

    if (mGOMap.HasElements())
    {
        if (mWarnOnNonemptyBridges)
        {
            LogWarning("Bridge map is nonempty on destruction.");
        }

        mGOMap.RemoveAll();
    }

    if (mDTMap.HasElements())
    {
        if (mWarnOnNonemptyBridges)
        {
            LogWarning("Draw target map nonempty on destruction.");
        }

        mDTMap.RemoveAll();
    }

    if (mILMap->HasElements())
    {
        if (mWarnOnNonemptyBridges)
        {
            LogWarning("Input layout map nonempty on destruction.");
        }

        mILMap->UnbindAll();
    }
    mILMap = nullptr;
}

uint64_t GL4Engine::DrawPrimitive(VertexBuffer const* vbuffer, IndexBuffer const* ibuffer)
{
    unsigned int numActiveVertices = vbuffer->GetNumActiveElements();
    unsigned int vertexOffset = vbuffer->GetOffset();

    unsigned int numActiveIndices = ibuffer->GetNumActiveIndices();
    unsigned int indexSize = ibuffer->GetElementSize();
    GLenum indexType = (indexSize == 4 ? GL_UNSIGNED_INT : GL_UNSIGNED_SHORT);

    GLenum topology = 0;
    IPType type = ibuffer->GetPrimitiveType();
    switch (type)
    {
    case IPType::IP_POLYPOINT:
        topology = GL_POINTS;
        break;
    case IPType::IP_POLYSEGMENT_DISJOINT:
        topology = GL_LINES;
        break;
    case IPType::IP_POLYSEGMENT_CONTIGUOUS:
        topology = GL_LINE_STRIP;
        break;
    case IPType::IP_TRIMESH:
        topology = GL_TRIANGLES;
        break;
    case IPType::IP_TRISTRIP:
        topology = GL_TRIANGLE_STRIP;
        break;
    case IPType::IP_POLYSEGMENT_DISJOINT_ADJ:
        topology = GL_LINES_ADJACENCY;
        break;
    case IPType::IP_POLYSEGMENT_CONTIGUOUS_ADJ:
        topology = GL_LINE_STRIP_ADJACENCY;
        break;
    case IPType::IP_TRIMESH_ADJ:
        topology = GL_TRIANGLES_ADJACENCY;
        break;
    case IPType::IP_TRISTRIP_ADJ:
        topology = GL_TRIANGLE_STRIP_ADJACENCY;
        break;
    default:
        LogError("Unknown primitive topology = " + std::to_string(type));
        return 0;
    }

    unsigned int offset = ibuffer->GetOffset();
    if (ibuffer->IsIndexed())
    {
        void const* data = (char*)0 + indexSize * offset;
        glDrawRangeElements(topology, 0, numActiveVertices - 1,
            static_cast<GLsizei>(numActiveIndices), indexType, data);
    }
    else
    {
        // From the OpenGL documentation on gl_VertexID vertex shader variable:
        // "gl_VertexID is a vertex language input variable that holds an integer index
        // for the vertex. The index is impliclty generated by glDrawArrays and other
        // commands that do not reference the content of the GL_ELEMENT_ARRAY_BUFFER,
        // or explicitly generated from the content of the GL_ELEMENT_ARRAY_BUFFER
        // by commands such as glDrawElements."
        glDrawArrays(topology, static_cast<GLint>(vertexOffset),
            static_cast<GLint>(numActiveVertices));
    }
    return 0;
}

bool GL4Engine::EnableShaders(std::shared_ptr<VisualEffect> const& effect, GLuint program)
{
    VertexShader* vshader = effect->GetVertexShader().get();
    if (!vshader)
    {
        LogError("Effect does not have a vertex shader.");
        return false;
    }

    PixelShader* pshader = effect->GetPixelShader().get();
    if (!pshader)
    {
        LogError("Effect does not have a pixel shader.");
        return false;
    }

    GeometryShader* gshader = effect->GetGeometryShader().get();

    // Enable the shader resources.
    Enable(vshader, program);
    Enable(pshader, program);
    if (gshader)
    {
        Enable(gshader, program);
    }

    return true;
}

void GL4Engine::DisableShaders(std::shared_ptr<VisualEffect> const& effect, GLuint program)
{
    VertexShader* vshader = effect->GetVertexShader().get();
    PixelShader* pshader = effect->GetPixelShader().get();
    GeometryShader* gshader = effect->GetGeometryShader().get();

    if (gshader)
    {
        Disable(gshader, program);
    }
    Disable(pshader, program);
    Disable(vshader, program);
}

void GL4Engine::Enable(Shader const* shader, GLuint program)
{
    EnableCBuffers(shader, program);
    EnableTBuffers(shader, program);
    EnableSBuffers(shader, program);
    EnableRBuffers(shader, program);
    EnableTextures(shader, program);
    EnableTextureArrays(shader, program);
    EnableSamplers(shader, program);
}

void GL4Engine::Disable(Shader const* shader, GLuint program)
{
    DisableSamplers(shader, program);
    DisableTextureArrays(shader, program);
    DisableTextures(shader, program);
    DisableRBuffers(shader, program);
    DisableSBuffers(shader, program);
    DisableTBuffers(shader, program);
    DisableCBuffers(shader, program);
}

void GL4Engine::EnableCBuffers(Shader const* shader, GLuint program)
{
    int const index = ConstantBuffer::shaderDataLookup;
    for (auto const& cb : shader->GetData(index))
    {
        if (cb.object)
        {
            auto gl4CB = static_cast<GL4ConstantBuffer*>(Bind(cb.object));
            if (gl4CB)
            {
                auto const blockIndex = cb.bindPoint;
                if (GL_INVALID_INDEX != blockIndex)
                {
                    auto const unit = mUniformUnitMap.AcquireUnit(program, blockIndex);
                    glUniformBlockBinding(program, blockIndex, unit);
                    gl4CB->AttachToUnit(unit);
                }
            }
            else
            {
                LogError("Failed to bind constant buffer.");
            }
        }
        else
        {
            LogError(cb.name + " is null constant buffer.");
        }
    }
}

void GL4Engine::DisableCBuffers(Shader const* shader, GLuint program)
{
    int const index = ConstantBuffer::shaderDataLookup;
    for (auto const& cb : shader->GetData(index))
    {
        auto const blockIndex = cb.bindPoint;
        if (GL_INVALID_INDEX != blockIndex)
        {
            auto const unit = mUniformUnitMap.GetUnit(program, blockIndex);
            glBindBufferBase(GL_UNIFORM_BUFFER, unit, 0);
            mUniformUnitMap.ReleaseUnit(unit);
        }
    }
}

void GL4Engine::EnableTBuffers(Shader const*, GLuint)
{
    // TODO
}

void GL4Engine::DisableTBuffers(Shader const*, GLuint)
{
    // TODO
}

void GL4Engine::EnableSBuffers(Shader const* shader, GLuint program)
{
    // Configure atomic counter buffer objects used by the shader.
    auto const& atomicCounters = shader->GetData(Shader::AtomicCounterShaderDataLookup);
    auto const& atomicCounterBuffers = shader->GetData(Shader::AtomicCounterBufferShaderDataLookup);
    for (unsigned acbIndex = 0; acbIndex < atomicCounterBuffers.size(); ++acbIndex)
    {
        auto const& acb = atomicCounterBuffers[acbIndex];

        // Allocate a new raw buffer?
        if (acbIndex >= mAtomicCounterRawBuffers.size())
        {
            mAtomicCounterRawBuffers.push_back(nullptr);
        }

        // Look at the current raw buffer defined at this index.
        // Could be nullptr if a new location was just inserted.
        auto& rawBuffer = mAtomicCounterRawBuffers[acbIndex];

        // If the raw buffer is not large enough, then unbind old one and
        // ready to create new one.
        if (rawBuffer && (acb.numBytes > static_cast<int>(rawBuffer->GetNumBytes())))
        {
            Unbind(rawBuffer.get());
            rawBuffer = nullptr;
        }

        // Find the currently mapped GL4AtomicCounterBuffer.
        GL4AtomicCounterBuffer* gl4ACB = nullptr;
        if (rawBuffer)
        {
            gl4ACB = static_cast<GL4AtomicCounterBuffer*>(Get(rawBuffer));
        }

        // Create a new buffer?
        else
        {
            // By definition, RawBuffer contains 4-byte elements.  Do not need
            // CPU side storage but must be able to copy values between buffers.
            rawBuffer = std::make_shared<RawBuffer>((acb.numBytes + 3) / 4, false);
            rawBuffer->SetUsage(Resource::DYNAMIC_UPDATE);

            // Manual Bind operation since this is a special mapping from
            // RawBuffer to GL4AtomicCounterBuffer.
            auto temp = GL4AtomicCounterBuffer::Create(mGEObjectCreator, rawBuffer.get());
            mGOMap.Insert(rawBuffer.get(), temp);
            gl4ACB = static_cast<GL4AtomicCounterBuffer*>(temp.get());
        }

        // TODO:
        // ShaderStorage blocks have a glShaderStorageBlockBinding() call
        // Uniform blocks have glUniforBlockBinding() call
        // Is there something equivalent for atomic counters buffers?

        // Bind this atomic counter buffer
        gl4ACB->AttachToUnit(acb.bindPoint);
    }

    int const indexSB = StructuredBuffer::shaderDataLookup;
    for (auto const& sb : shader->GetData(indexSB))
    {
        if (sb.object)
        {
            auto gl4SB = static_cast<GL4StructuredBuffer*>(Bind(sb.object));
            if (gl4SB)
            {
                auto const blockIndex = sb.bindPoint;
                if (GL_INVALID_INDEX != blockIndex)
                {
                    auto const unit = mShaderStorageUnitMap.AcquireUnit(program, blockIndex);
                    glShaderStorageBlockBinding(program, blockIndex, unit);

                    // Do not use glBindBufferBase here.  Use AttachToUnit
                    // method in GL4StructuredBuffer.
                    gl4SB->AttachToUnit(unit);

                    // The sb.isGpuWritable flag is used to indicate whether
                    // or not there is atomic counter associated with this
                    // structured buffer.
                    if (sb.isGpuWritable)
                    {
                        // Does the structured buffer counter need to be reset?
                        gl4SB->SetNumActiveElements();

                        // This structured buffer has index to associated
                        // atomic counter table entry.
                        auto const acIndex = sb.extra;

                        // Where does the associated counter exist in the shader?
                        auto const acbIndex = atomicCounters[acIndex].bindPoint;
                        auto const acbOffset = atomicCounters[acIndex].extra;

                        // Retrieve the GL4 atomic counter buffer object.
                        auto gl4ACB = static_cast<GL4AtomicCounterBuffer*>(Get(mAtomicCounterRawBuffers[acbIndex]));

                        // Copy the counter value from the structured buffer object
                        // to the appropriate place in the atomic counter buffer.
                        gl4SB->CopyCounterValueToBuffer(gl4ACB, acbOffset);
                    }
                }
            }
            else
            {
                LogError("Failed to bind structured buffer.");
            }
        }
        else
        {
            LogError(sb.name + " is null structured buffer.");
        }
    }
}

void GL4Engine::DisableSBuffers(Shader const* shader, GLuint program)
{
    // Unbind any atomic counter buffers.
    auto const& atomicCounters = shader->GetData(Shader::AtomicCounterShaderDataLookup);
    auto const& atomicCounterBuffers = shader->GetData(Shader::AtomicCounterBufferShaderDataLookup);
    for (unsigned acbIndex = 0; acbIndex < atomicCounterBuffers.size(); ++acbIndex)
    {
        auto const& acb = atomicCounterBuffers[acbIndex];
        glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, acb.bindPoint, 0);
    }

    int const index = StructuredBuffer::shaderDataLookup;
    for (auto const& sb : shader->GetData(index))
    {
        if (sb.object)
        {
            auto gl4SB = static_cast<GL4StructuredBuffer*>(Get(sb.object));

            if (gl4SB)
            {
                auto const blockIndex = sb.bindPoint;
                if (GL_INVALID_INDEX != blockIndex)
                {
                    auto const unit = mShaderStorageUnitMap.GetUnit(program, blockIndex);
                    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, unit, 0);
                    mShaderStorageUnitMap.ReleaseUnit(unit);

                    // The sb.isGpuWritable flag is used to indicate whether
                    // or not there is atomic counter associated with this
                    // structured buffer.
                    if (sb.isGpuWritable)
                    {
                        // This structured buffer has index to associated
                        // atomic counter table entry.
                        auto const acIndex = sb.extra;

                        // Where does the associated counter exist in the shader?
                        auto const acbIndex = atomicCounters[acIndex].bindPoint;
                        auto const acbOffset = atomicCounters[acIndex].extra;

                        // Retrieve the GL4 atomic counter buffer object.
                        auto gl4ACB = static_cast<GL4AtomicCounterBuffer*>(Get(mAtomicCounterRawBuffers[acbIndex]));

                        // Copy the counter value from the appropriate place
                        // in the atomic counter buffer to the structured buffer
                        // object.
                        gl4SB->CopyCounterValueFromBuffer(gl4ACB, acbOffset);
                    }
                }
            }
        }
    }
}

void GL4Engine::EnableRBuffers(Shader const*, GLuint)
{
    // TODO
}

void GL4Engine::DisableRBuffers(Shader const*, GLuint)
{
    // TODO
}

void GL4Engine::EnableTextures(Shader const* shader, GLuint program)
{
    int const index = TextureSingle::shaderDataLookup;
    for (auto const& ts : shader->GetData(index))
    {
        if (!ts.object)
        {
            LogError(ts.name + " is null texture.");
            continue;
        }

        auto texture = static_cast<GL4TextureSingle*>(Bind(ts.object));
        if (!texture)
        {
            LogError("Failed to bind texture.");
            continue;
        }

        // By convension, ts.isGpuWritable is true for "image*" and false
        // for "sampler*".
        GLuint handle = texture->GetGLHandle();
        if (ts.isGpuWritable)
        {
            // For "image*" objects in the shader, use "readonly" or
            // "writeonly" attributes in the layout to control R/W/RW access
            // using shader compiler and then connect as GL_READ_WRITE here.
            // Always bind level 0 and all layers.
            GLint unit = mTextureImageUnitMap.AcquireUnit(program, ts.bindPoint);
            glUniform1i(ts.bindPoint, unit);
            DFType format = texture->GetTexture()->GetFormat();
            GLuint internalFormat = texture->GetInternalFormat(format);
            glBindImageTexture(unit, handle, 0, GL_TRUE, 0, GL_READ_WRITE, internalFormat);
        }
        else
        {
            GLint unit = mTextureSamplerUnitMap.AcquireUnit(program, ts.bindPoint);
            glUniform1i(ts.bindPoint, unit);
            glActiveTexture(GL_TEXTURE0 + unit);
            glBindTexture(texture->GetTarget(), handle);
        }
    }
}

void GL4Engine::DisableTextures(Shader const* shader, GLuint program)
{
    int const index = TextureSingle::shaderDataLookup;
    for (auto const& ts : shader->GetData(index))
    {
        if (!ts.object)
        {
            LogError(ts.name + " is null texture.");
            continue;
        }

        auto texture = static_cast<GL4TextureSingle*>(Get(ts.object));
        if (!texture)
        {
            LogError("Failed to get texture.");
            continue;
        }

        // By convension, ts.isGpuWritable is true for "image*" and false
        // for "sampler*".
        if (ts.isGpuWritable)
        {
            // For "image*" objects in the shader, use "readonly" or
            // "writeonly" attributes in the layout to control R/W/RW access
            // using shader compiler and then connect as GL_READ_WRITE here.
            // Always bind level 0 and all layers.  TODO: Decide if unbinding
            // the texture from the image unit is necessary.
            // glBindImageTexture(unit, 0, 0, 0, 0, 0, 0);
            GLint unit = mTextureImageUnitMap.GetUnit(program, ts.bindPoint);
            mTextureImageUnitMap.ReleaseUnit(unit);
        }
        else
        {
            GLint unit = mTextureSamplerUnitMap.GetUnit(program, ts.bindPoint);
            glActiveTexture(GL_TEXTURE0 + unit);
            glBindTexture(texture->GetTarget(), 0);
            mTextureSamplerUnitMap.ReleaseUnit(unit);
        }
    }
}

void GL4Engine::EnableTextureArrays(Shader const* shader, GLuint program)
{
    int const index = TextureArray::shaderDataLookup;
    for (auto const& ta : shader->GetData(index))
    {
        if (!ta.object)
        {
            LogError(ta.name + " is null texture array.");
            continue;
        }

        auto texture = static_cast<GL4TextureArray*>(Bind(ta.object));
        if (!texture)
        {
            LogError("Failed to bind texture array.");
            continue;
        }

        // By convension, ta.isGpuWritable is true for "image*" and false
        // for "sampler*".
        GLuint handle = texture->GetGLHandle();
        if (ta.isGpuWritable)
        {
            // For "image*" objects in the shader, use "readonly" or
            // "writeonly" attributes in the layout to control R/W/RW access
            // using shader compiler and then connect as GL_READ_WRITE here.
            // Always bind level 0 and all layers.
            GLint unit = mTextureImageUnitMap.AcquireUnit(program, ta.bindPoint);
            glUniform1i(ta.bindPoint, unit);
            DFType format = texture->GetTexture()->GetFormat();
            GLuint internalFormat = texture->GetInternalFormat(format);
            glBindImageTexture(unit, handle, 0, GL_TRUE, 0, GL_READ_WRITE, internalFormat);
        }
        else
        {
            GLint unit = mTextureSamplerUnitMap.AcquireUnit(program, ta.bindPoint);
            glUniform1i(ta.bindPoint, unit);
            glActiveTexture(GL_TEXTURE0 + unit);
            glBindTexture(texture->GetTarget(), handle);
        }
    }
}

void GL4Engine::DisableTextureArrays(Shader const* shader, GLuint program)
{
    int const index = TextureArray::shaderDataLookup;
    for (auto const& ta : shader->GetData(index))
    {
        if (!ta.object)
        {
            LogError(ta.name + " is null texture array.");
            continue;
        }

        auto texture = static_cast<GL4TextureArray*>(Get(ta.object));
        if (!texture)
        {
            LogError("Failed to get texture array.");
            continue;
        }

        // By convension, ta.isGpuWritable is true for "image*" and false
        // for "sampler*".
        if (ta.isGpuWritable)
        {
            // For "image*" objects in the shader, use "readonly" or
            // "writeonly" attributes in the layout to control R/W/RW access
            // using shader compiler and then connect as GL_READ_WRITE here.
            // Always bind level 0 and all layers.  TODO: Decide if unbinding
            // the texture from the image unit is necessary.
            // glBindImageTexture(unit, 0, 0, 0, 0, 0, 0);
            GLint unit = mTextureImageUnitMap.GetUnit(program, ta.bindPoint);
            mTextureImageUnitMap.ReleaseUnit(unit);
        }
        else
        {
            GLint unit = mTextureSamplerUnitMap.GetUnit(program, ta.bindPoint);
            glActiveTexture(GL_TEXTURE0 + unit);
            glBindTexture(texture->GetTarget(), 0);
            mTextureSamplerUnitMap.ReleaseUnit(unit);
        }
    }
}

void GL4Engine::EnableSamplers(Shader const* shader, GLuint program)
{
    int const index = SamplerState::shaderDataLookup;
    for (auto const& ts : shader->GetData(index))
    {
        if (ts.object)
        {
            auto gl4Sampler = static_cast<GL4SamplerState*>(Bind(ts.object));
            if (gl4Sampler)
            {
                auto const location = ts.bindPoint;
                auto const unit = mTextureSamplerUnitMap.AcquireUnit(program, location);
                glBindSampler(unit, gl4Sampler->GetGLHandle());
            }
            else
            {
                LogError("Failed to bind sampler.");
            }
        }
        else
        {
            LogError(ts.name + " is null sampler.");
        }
    }
}

void GL4Engine::DisableSamplers(Shader const* shader, GLuint program)
{
    int const index = SamplerState::shaderDataLookup;
    for (auto const& ts : shader->GetData(index))
    {
        if (ts.object)
        {
            auto gl4Sampler = static_cast<GL4SamplerState*>(Get(ts.object));

            if (gl4Sampler)
            {
                auto const location = ts.bindPoint;
                auto const unit = mTextureSamplerUnitMap.GetUnit(program, location);
                glBindSampler(unit, 0);
                mTextureSamplerUnitMap.ReleaseUnit(unit);
            }
            else
            {
                LogError("Failed to get sampler.");
            }
        }
        else
        {
            LogError(ts.name + " is null sampler.");
        }
    }
}

GL4Engine::ProgramIndexUnitMap::~ProgramIndexUnitMap()
{
}

GL4Engine::ProgramIndexUnitMap::ProgramIndexUnitMap()
{
}

int GL4Engine::ProgramIndexUnitMap::AcquireUnit(GLint program, GLint index)
{
    int availUnit = -1;
    for (int unit = 0; unit < static_cast<int>(mLinkMap.size()); ++unit)
    {
        auto& item = mLinkMap[unit];

        // Increment link count if already assigned and in use?
        if (program == item.program && index == item.index)
        {
            ++item.linkCount;
            return unit;
        }

        // Found a unit that was previously used but is now avaialble.
        if (0 == item.linkCount)
        {
            if (-1 == availUnit)
            {
                availUnit = unit;
            }
        }
    }

    // New unit number not previously used?
    if (-1 == availUnit)
    {
        // TODO: Consider querying the max number of units
        // and check that this number is not exceeded.
        availUnit = static_cast<int>(mLinkMap.size());
        mLinkMap.push_back({ 0, 0, 0 });
    }

    auto& item = mLinkMap[availUnit];
    item.linkCount = 1;
    item.program = program;
    item.index = index;
    return availUnit;
}

int GL4Engine::ProgramIndexUnitMap::GetUnit(GLint program, GLint index) const
{
    for (int unit = 0; unit < static_cast<int>(mLinkMap.size()); ++unit)
    {
        auto& item = mLinkMap[unit];
        if (program == item.program && index == item.index)
        {
            return unit;
        }
    }
    return -1;
}

void GL4Engine::ProgramIndexUnitMap::ReleaseUnit(unsigned index)
{
    if (index < mLinkMap.size())
    {
        auto& item = mLinkMap[index];
        if (item.linkCount > 0)
        {
            --item.linkCount;
        }
    }
}

unsigned GL4Engine::ProgramIndexUnitMap::GetUnitLinkCount(unsigned unit) const
{
    if (unit < mLinkMap.size())
    {
        return mLinkMap[unit].linkCount;
    }
    return 0;
}

bool GL4Engine::ProgramIndexUnitMap::GetUnitProgramIndex(unsigned unit, GLint &program, GLint &index) const
{
    if (unit < mLinkMap.size())
    {
        auto& item = mLinkMap[index];
        if (item.linkCount > 0)
        {
            program = item.program;
            index = item.index;
            return true;
        }
    }
    return false;
}

//----------------------------------------------------------------------------
// Overrides from GraphicsEngine
//----------------------------------------------------------------------------
void GL4Engine::SetViewport(int x, int y, int w, int h)
{
    glViewport(x, y, w, h);
}

void GL4Engine::GetViewport(int& x, int& y, int& w, int& h) const
{
    int param[4];
    glGetIntegerv(GL_VIEWPORT, param);
    x = param[0];
    y = param[1];
    w = param[2];
    h = param[3];
}

void GL4Engine::SetDepthRange(float zmin, float zmax)
{
    glDepthRange(static_cast<GLdouble>(zmin), static_cast<GLdouble>(zmax));
}

void GL4Engine::GetDepthRange(float& zmin, float& zmax) const
{
    GLdouble param[2];
    glGetDoublev(GL_DEPTH_RANGE, param);
    zmin = static_cast<float>(param[0]);
    zmax = static_cast<float>(param[1]);
}

bool GL4Engine::Resize(unsigned int w, unsigned int h)
{
    mXSize = w;
    mYSize = h;
    int param[4];
    glGetIntegerv(GL_VIEWPORT, param);
    glViewport(param[0], param[1], static_cast<GLint>(w), static_cast<GLint>(h));
    return true;
}

void GL4Engine::ClearColorBuffer()
{
    glClearColor(mClearColor[0], mClearColor[1], mClearColor[2], mClearColor[3]);
    glClear(GL_COLOR_BUFFER_BIT);
}

void GL4Engine::ClearDepthBuffer()
{
    glClearDepth(mClearDepth);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void GL4Engine::ClearStencilBuffer()
{
    glClearStencil(static_cast<GLint>(mClearStencil));
    glClear(GL_STENCIL_BUFFER_BIT);
}

void GL4Engine::ClearBuffers()
{
    glClearColor(mClearColor[0], mClearColor[1], mClearColor[2], mClearColor[3]);
    glClearDepth(mClearDepth);
    glClearStencil(static_cast<GLint>(mClearStencil));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void GL4Engine::SetBlendState(std::shared_ptr<BlendState> const& state)
{
    if (state)
    {
        if (state != mActiveBlendState)
        {
            GL4BlendState* gl4State = static_cast<GL4BlendState*>(Bind(state));
            if (gl4State)
            {
                gl4State->Enable();
                mActiveBlendState = state;
            }
            else
            {
                LogError("Failed to bind blend state.");
            }
        }
    }
    else
    {
        LogError("Input state is null.");
    }
}

void GL4Engine::SetDepthStencilState(std::shared_ptr<DepthStencilState> const& state)
{
    if (state)
    {
        if (state != mActiveDepthStencilState)
        {
            GL4DepthStencilState* gl4State = static_cast<GL4DepthStencilState*>(Bind(state));
            if (gl4State)
            {
                gl4State->Enable();
                mActiveDepthStencilState = state;
            }
            else
            {
                LogError("Failed to bind depth-stencil state.");
            }
        }
    }
    else
    {
        LogError("Input state is null.");
    }
}

void GL4Engine::SetRasterizerState(std::shared_ptr<RasterizerState> const& state)
{
    if (state)
    {
        if (state != mActiveRasterizerState)
        {
            GL4RasterizerState* gl4State = static_cast<GL4RasterizerState*>(Bind(state));
            if (gl4State)
            {
                gl4State->Enable();
                mActiveRasterizerState = state;
            }
            else
            {
                LogError("Failed to bind rasterizer state.");
            }
        }
    }
    else
    {
        LogError("Input state is null.");
    }
}

void GL4Engine::Enable(std::shared_ptr<DrawTarget> const& target)
{
    auto gl4Target = static_cast<GL4DrawTarget*>(Bind(target));
    gl4Target->Enable();
}

void GL4Engine::Disable(std::shared_ptr<DrawTarget> const& target)
{
    auto gl4Target = static_cast<GL4DrawTarget*>(Get(target));
    if (gl4Target)
    {
        gl4Target->Disable();
    }
}

bool GL4Engine::Update(std::shared_ptr<Buffer> const& buffer)
{
    if (!buffer->GetData())
    {
        LogWarning("Buffer does not have system memory, creating it.");
        buffer->CreateStorage();
    }

    auto glBuffer = static_cast<GL4Buffer*>(Bind(buffer));
    return glBuffer->Update();
}

bool GL4Engine::Update(std::shared_ptr<TextureSingle> const& texture)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->Update();
}

bool GL4Engine::Update(std::shared_ptr<TextureSingle> const& texture, unsigned int level)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->Update(level);
}

bool GL4Engine::Update(std::shared_ptr<TextureArray> const& textureArray)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->Update();
}

bool GL4Engine::Update(std::shared_ptr<TextureArray> const& textureArray, unsigned int item, unsigned int level)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->Update(item, level);
}

bool GL4Engine::CopyCpuToGpu(std::shared_ptr<Buffer> const& buffer)
{
    if (!buffer->GetData())
    {
        LogWarning("Buffer does not have system memory, creating it.");
        buffer->CreateStorage();
    }

    auto glBuffer = static_cast<GL4Buffer*>(Bind(buffer));
    return glBuffer->CopyCpuToGpu();
}

bool GL4Engine::CopyCpuToGpu(std::shared_ptr<TextureSingle> const& texture)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->CopyCpuToGpu();
}

bool GL4Engine::CopyCpuToGpu(std::shared_ptr<TextureSingle> const& texture, unsigned int level)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->CopyCpuToGpu(level);
}

bool GL4Engine::CopyCpuToGpu(std::shared_ptr<TextureArray> const& textureArray)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->CopyCpuToGpu();
}

bool GL4Engine::CopyCpuToGpu(std::shared_ptr<TextureArray> const& textureArray, unsigned int item, unsigned int level)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->CopyCpuToGpu(item, level);
}

bool GL4Engine::CopyGpuToCpu(std::shared_ptr<Buffer> const& buffer)
{
    if (!buffer->GetData())
    {
        LogWarning("Buffer does not have system memory, creating it.");
        buffer->CreateStorage();
    }

    auto glBuffer = static_cast<GL4Buffer*>(Bind(buffer));
    return glBuffer->CopyGpuToCpu();
}

bool GL4Engine::CopyGpuToCpu(std::shared_ptr<TextureSingle> const& texture)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->CopyGpuToCpu();
}

bool GL4Engine::CopyGpuToCpu(std::shared_ptr<TextureSingle> const& texture, unsigned int level)
{
    if (!texture->GetData())
    {
        LogWarning("Texture does not have system memory, creating it.");
        texture->CreateStorage();
    }

    auto glTexture = static_cast<GL4TextureSingle*>(Bind(texture));
    return glTexture->CopyGpuToCpu(level);
}

bool GL4Engine::CopyGpuToCpu(std::shared_ptr<TextureArray> const& textureArray)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->CopyGpuToCpu();
}

bool GL4Engine::CopyGpuToCpu(std::shared_ptr<TextureArray> const& textureArray, unsigned int item, unsigned int level)
{
    if (!textureArray->GetData())
    {
        LogWarning("Texture array does not have system memory, creating it.");
        textureArray->CreateStorage();
    }

    auto glTextureArray = static_cast<GL4TextureArray*>(Bind(textureArray));
    return glTextureArray->CopyGpuToCpu(item, level);
}

void GL4Engine::CopyGpuToGpu(
    std::shared_ptr<Buffer> const& buffer0,
    std::shared_ptr<Buffer> const& buffer1)
{
    (void)buffer0;
    (void)buffer1;
    LogError("Not yet implemented.");
}

void GL4Engine::CopyGpuToGpu(
    std::shared_ptr<TextureSingle> const& texture0,
    std::shared_ptr<TextureSingle> const& texture1)
{
    (void)texture0;
    (void)texture1;
    LogError("Not yet implemented.");
}

void GL4Engine::CopyGpuToGpu(
    std::shared_ptr<TextureSingle> const& texture0,
    std::shared_ptr<TextureSingle> const& texture1,
    unsigned int level)
{
    (void)texture0;
    (void)texture1;
    (void)level;
    LogError("Not yet implemented.");
}

void GL4Engine::CopyGpuToGpu(
    std::shared_ptr<TextureArray> const& textureArray0,
    std::shared_ptr<TextureArray> const& textureArray1)
{
    (void)textureArray0;
    (void)textureArray1;
    LogError("Not yet implemented.");
}

void GL4Engine::CopyGpuToGpu(
    std::shared_ptr<TextureArray> const& textureArray0,
    std::shared_ptr<TextureArray> const& textureArray1,
    unsigned int item, unsigned int level)
{
    (void)textureArray0;
    (void)textureArray1;
    (void)item;
    (void)level;
    LogError("Not yet implemented.");
}

bool GL4Engine::GetNumActiveElements(std::shared_ptr<StructuredBuffer> const& buffer)
{
    auto gl4Object = Get(buffer);
    if (gl4Object)
    {
        auto gl4SBuffer = static_cast<GL4StructuredBuffer*>(gl4Object);
        return gl4SBuffer->GetNumActiveElements();
    }
    return false;
}

bool GL4Engine::BindProgram(std::shared_ptr<ComputeProgram> const&)
{
    // TODO: Why are we not adding the compute shader to the mGOMap?
    return true;
}

void GL4Engine::Execute(std::shared_ptr<ComputeProgram> const& program,
    unsigned int numXGroups, unsigned int numYGroups, unsigned int numZGroups)
{
    auto glslProgram = std::dynamic_pointer_cast<GLSLComputeProgram>(program);
    if (glslProgram && numXGroups > 0 && numYGroups > 0 && numZGroups > 0)
    {
        auto cshader = glslProgram->GetCShader();
        auto programHandle = glslProgram->GetProgramHandle();
        if (cshader && programHandle > 0)
        {
            glUseProgram(programHandle);
            Enable(cshader.get(), programHandle);
            glDispatchCompute(numXGroups, numYGroups, numZGroups);
            Disable(cshader.get(), programHandle);
            glUseProgram(0);
        }
    }
    else
    {
        LogError("Invalid input parameter.");
    }
}

void GL4Engine::WaitForFinish()
{
    // TODO.  Determine whether OpenGL can wait for a compute program to finish.
}

void GL4Engine::Flush()
{
    glFlush();
}

uint64_t GL4Engine::DrawPrimitive(std::shared_ptr<VertexBuffer> const& vbuffer,
    std::shared_ptr<IndexBuffer> const& ibuffer, std::shared_ptr<VisualEffect> const& effect)
{
    GLSLVisualProgram* gl4program = dynamic_cast<GLSLVisualProgram*>(effect->GetProgram().get());
    if (!gl4program)
    {
        LogError("HLSL effect passed to GLSL engine.");
        return 0;
    }

    uint64_t numPixelsDrawn = 0;
    auto programHandle = gl4program->GetProgramHandle();
    glUseProgram(programHandle);

    if (EnableShaders(effect, programHandle))
    {
        // Enable the vertex buffer and input layout.
        GL4VertexBuffer* gl4VBuffer = nullptr;
        GL4InputLayout* gl4Layout = nullptr;
        if (vbuffer->StandardUsage())
        {
            gl4VBuffer = static_cast<GL4VertexBuffer*>(Bind(vbuffer));
            GL4InputLayoutManager* manager = static_cast<GL4InputLayoutManager*>(mILMap.get());
            gl4Layout = manager->Bind(programHandle, gl4VBuffer->GetGLHandle(), vbuffer.get());
            gl4Layout->Enable();
        }

        // Enable the index buffer.
        GL4IndexBuffer* gl4IBuffer = nullptr;
        if (ibuffer->IsIndexed())
        {
            gl4IBuffer = static_cast<GL4IndexBuffer*>(Bind(ibuffer));
            gl4IBuffer->Enable();
        }

        numPixelsDrawn = DrawPrimitive(vbuffer.get(), ibuffer.get());

        // Disable the vertex buffer and input layout.
        if (vbuffer->StandardUsage())
        {
            gl4Layout->Disable();
        }

        // Disable the index buffer.
        if (gl4IBuffer)
        {
            gl4IBuffer->Disable();
        }

        DisableShaders(effect, programHandle);
    }

    glUseProgram(0);

    return numPixelsDrawn;
}
