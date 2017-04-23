// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <Graphics/GteShader.h>
using namespace gte;


Shader::Data::Data(GraphicsObjectType inType, std::string const& inName,
    int inBindPoint, int inNumBytes, unsigned int inExtra,
    bool inIsGpuWritable)
    :
    type(inType),
    name(inName),
    bindPoint(inBindPoint),
    numBytes(inNumBytes),
    extra(inExtra),
    isGpuWritable(inIsGpuWritable)
{
}

#if defined(GTE_DEV_OPENGL)
Shader::Shader(GLSLReflection const& reflector, int type)
    :
    mNumXThreads(0),  // TODO: compute shader support
    mNumYThreads(0),
    mNumZThreads(0)
{
    // If this is a compute shader, then query the number of threads per group.
    if (GLSLReflection::ST_COMPUTE == type)
    {
        GLint sizeX, sizeY, sizeZ;
        reflector.GetComputeShaderWorkGroupSize(sizeX, sizeY, sizeZ);
        mNumXThreads = sizeX;
        mNumYThreads = sizeY;
        mNumZThreads = sizeZ;
    }

    // Will need to access uniforms more than once.
    auto const& uniforms = reflector.GetUniforms();

    // Gather the uninforms information to create texture data.
    for (auto const& uni : uniforms)
    {
        if (uni.referencedBy[type])
        {
            // Only interested in these particular uniform types (for gsampler* and gimage*)
            switch (uni.type)
            {
                case GL_SAMPLER_1D:
                case GL_INT_SAMPLER_1D:
                case GL_UNSIGNED_INT_SAMPLER_1D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 1, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE1, false));
                    break;

                case GL_SAMPLER_2D:
                case GL_INT_SAMPLER_2D:
                case GL_UNSIGNED_INT_SAMPLER_2D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 2, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE2, false));
                    break;

                case GL_SAMPLER_3D:
                case GL_INT_SAMPLER_3D:
                case GL_UNSIGNED_INT_SAMPLER_3D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 3, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE3, false));
                    break;

                case GL_SAMPLER_1D_ARRAY:
                case GL_INT_SAMPLER_1D_ARRAY:
                case GL_UNSIGNED_INT_SAMPLER_1D_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 1, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE1_ARRAY, false));
                    break;

                case GL_SAMPLER_2D_ARRAY:
                case GL_INT_SAMPLER_2D_ARRAY:
                case GL_UNSIGNED_INT_SAMPLER_2D_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 2, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE2_ARRAY, false));
                    break;

                case GL_SAMPLER_CUBE:
                case GL_INT_SAMPLER_CUBE:
                case GL_UNSIGNED_INT_SAMPLER_CUBE:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 2, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE_CUBE, false));
                    break;

                case GL_SAMPLER_CUBE_MAP_ARRAY:
                case GL_INT_SAMPLER_CUBE_MAP_ARRAY:
                case GL_UNSIGNED_INT_SAMPLER_CUBE_MAP_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 3, false));
                    mData[SamplerState::shaderDataLookup].push_back(
                        Data(GT_SAMPLER_STATE, uni.name, uni.location, 0, GT_TEXTURE_CUBE_ARRAY, false));
                    break;

                case GL_IMAGE_1D:
                case GL_INT_IMAGE_1D:
                case GL_UNSIGNED_INT_IMAGE_1D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 1, true));
                    break;

                case GL_IMAGE_2D:
                case GL_INT_IMAGE_2D:
                case GL_UNSIGNED_INT_IMAGE_2D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 2, true));
                    break;

                case GL_IMAGE_3D:
                case GL_INT_IMAGE_3D:
                case GL_UNSIGNED_INT_IMAGE_3D:
                    mData[TextureSingle::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_SINGLE, uni.name, uni.location, 0, 3, true));
                    break;

                case GL_IMAGE_1D_ARRAY:
                case GL_INT_IMAGE_1D_ARRAY:
                case GL_UNSIGNED_INT_IMAGE_1D_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 1, true));
                    break;

                case GL_IMAGE_2D_ARRAY:
                case GL_INT_IMAGE_2D_ARRAY:
                case GL_UNSIGNED_INT_IMAGE_2D_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 2, true));
                    break;

                case GL_IMAGE_CUBE:
                case GL_INT_IMAGE_CUBE:
                case GL_UNSIGNED_INT_IMAGE_CUBE:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 2, true));
                    break;

                case GL_IMAGE_CUBE_MAP_ARRAY:
                case GL_INT_IMAGE_CUBE_MAP_ARRAY:
                case GL_UNSIGNED_INT_IMAGE_CUBE_MAP_ARRAY:
                    mData[TextureArray::shaderDataLookup].push_back(
                        Data(GT_TEXTURE_ARRAY, uni.name, uni.location, 0, 3, true));
                    break;
            }
        }
    }

    // Gather the uniform blocks information to create constant buffer data.
    auto const& uniformBlocks = reflector.GetUniformBlocks();
    int numUniformBlockReferences = 0;
    for (auto const& block : uniformBlocks)
    {
        if (block.referencedBy[type])
        {
            ++numUniformBlockReferences;
        }
    }
    if (numUniformBlockReferences > 0)
    {
        mCBufferLayouts.resize(numUniformBlockReferences);

        // Store information needed by GL4Engine for enabling/disabling the
        // constant buffers.
        int blockIndex = 0;
        int layoutIndex = 0;
        for (auto const& block : uniformBlocks)
        {
            if (block.referencedBy[type])
            {
                mData[ConstantBuffer::shaderDataLookup].push_back(
                    Data(GT_CONSTANT_BUFFER, block.name, block.bufferBinding,
                    block.bufferDataSize, 0, false));

                // Assemble the constant buffer layout information.
                for (auto const& uniform : uniforms)
                {
                    if (uniform.blockIndex != blockIndex)
                    {
                        continue;

                    }
                    MemberLayout item;
                    item.name = uniform.name;
                    item.offset = uniform.offset;
                    // TODO: The HLSL reflection has numElements of 0 when
                    // the item is not an array, but the actual number when
                    // it is an array.  ConstantBuffer::SetMember(...) uses
                    // this information, so we need to adhere to the pattern.
                    // Change this design in a refactor?
                    item.numElements =
                        (uniform.arraySize > 1 ? uniform.arraySize : 0);

                    mCBufferLayouts[layoutIndex].push_back(item);
                }

                ++layoutIndex;
            }

            ++blockIndex;
        }
    }

    // Gather the atomic counter buffer information to create atomic counter buffer data.
    auto const& atomicCounterBuffers = reflector.GetAtomicCounterBuffers();
    int numAtomicCounterBufferReferences = 0;
    for (auto const& block : atomicCounterBuffers)
    {
        if (block.referencedBy[type])
        {
            ++numAtomicCounterBufferReferences;
        }
    }
    if (numAtomicCounterBufferReferences > 0)
    {
        unsigned blockIndex = 0;
        for (auto const& block : atomicCounterBuffers)
        {
            if (block.referencedBy[type])
            {
                // It is possible for the atomic counter buffer to indicate it
                // only has 4 bytes for a single counter located at offset=4.
                // But we will want to allocate a buffer large enough to store
                // from offset=0 to the last counter declared in the buffer.
                unsigned bufferDataSize = block.bufferDataSize;
                for (unsigned i=0; i < block.activeVariables.size(); ++i)
                {
                    auto const& ac = uniforms[block.activeVariables[i]];
                    unsigned const lastByte = ac.offset + 4;

                    bufferDataSize = std::max(bufferDataSize, lastByte);
                }

                mData[AtomicCounterBufferShaderDataLookup].push_back(
                    Data(GT_RESOURCE, "atomicCounterBuffer" + std::to_string(blockIndex),
                    block.bufferBinding, bufferDataSize, 0, true));
            }
            ++blockIndex;
        }
    }

    // Gather the buffer blocks information to create structured buffer data.
    auto const& bufferBlocks = reflector.GetBufferBlocks();
    int numBufferBlockReferences = 0;
    for (auto const& block : bufferBlocks)
    {
        if (block.referencedBy[type])
        {
            ++numBufferBlockReferences;
        }
    }
    if (numBufferBlockReferences > 0)
    {
        auto const& bufferVariables = reflector.GetBufferVariables();
        mSBufferLayouts.resize(numBufferBlockReferences);

        // Store information needed by GL4Engine for enabling/disabling the
        // structured buffers.
        int blockIndex = 0;
        int layoutIndex = 0;
        for (auto const& block : bufferBlocks)
        {
            if (block.referencedBy[type])
            {
                // Search through uniforms looking for atomic counter with
                // the same name and "Counter" suffix.  The ID is the index
                // for this uniform so that it can be looked up later.
                auto const counterName = block.name + "Counter";
                bool hasAtomicCounter = false;
                unsigned int idAtomicCounter = ~0U;
                for (auto const& uniform : uniforms)
                {
                    if ((counterName == uniform.name) && (uniform.atomicCounterBufferIndex >= 0))
                    {
                        hasAtomicCounter = true;
                        idAtomicCounter = static_cast<unsigned int>(mData[AtomicCounterShaderDataLookup].size());
                        mData[AtomicCounterShaderDataLookup].push_back(
                            Data(GT_STRUCTURED_BUFFER, uniform.name, uniform.atomicCounterBufferIndex,
                            4, uniform.offset, false));
                        break;
                    }
                }

                // Assemble the structured buffer layout information.
                // Only interested in variables in the buffer that are part of
                // a top level array stride.  Anything up to this block is ignored
                // and anything after this block is ignored which means only one
                // top level array is supported.
                auto& layout = mSBufferLayouts[layoutIndex];
                GLint structSize = 0;
                for (unsigned v = 0; v < block.activeVariables.size(); ++v)
                {
                    auto const& bufferVar = bufferVariables[block.activeVariables[v]];

                    if (bufferVar.topLevelArrayStride != structSize)
                    {
                        // Stop when we were processing buffer variables with a certain
                        // a top-level array stride and that changed.
                        if (0 != structSize)
                        {
                            break;
                        }
                        structSize = bufferVar.topLevelArrayStride;
                    }

                    // These are the variables in the structured buffer.
                    if (structSize > 0)
                    {
                        MemberLayout item;
                        item.name = bufferVar.name;
                        item.offset = bufferVar.offset;
                        // TODO: The HLSL reflection has numElements of 0 when
                        // the item is not an array, but the actual number when
                        // it is an array.  ConstantBuffer::SetMember(...) uses
                        // this information, so we need to adhere to the pattern.
                        // Change this design in a refactor?
                        item.numElements =
                            (bufferVar.arraySize > 1 ? bufferVar.arraySize : 0);

                        layout.push_back(item);
                    }
                }

                // Use the top level array stride as a better indication
                // of the overall struct size.
                mData[StructuredBuffer::shaderDataLookup].push_back(
                    Data(GT_STRUCTURED_BUFFER, block.name, block.bufferBinding,
                    structSize, idAtomicCounter, hasAtomicCounter));

                ++layoutIndex;
            }

            ++blockIndex;
        }
    }

    // The conversion depends on the 'type' of the ordering:  {vertex = 0,
    // geometry = 1, pixel = 2, compute = 3}.
    mType = static_cast<GraphicsObjectType>(GT_SHADER + 1 + type);
}
#else
Shader::Shader(HLSLShader const& program)
    :
    mCompiledCode(program.GetCompiledCode()),
    mNumXThreads(program.GetNumXThreads()),
    mNumYThreads(program.GetNumYThreads()),
    mNumZThreads(program.GetNumZThreads())
{
    mCBufferLayouts.resize(program.GetCBuffers().size());
    int i = 0;
    for (auto const& cb : program.GetCBuffers())
    {
        mData[ConstantBuffer::shaderDataLookup].push_back(
            Data(GT_CONSTANT_BUFFER, cb.GetName(), cb.GetBindPoint(),
                cb.GetNumBytes(), 0, false));

        cb.GenerateLayout(mCBufferLayouts[i]);
        ++i;
    }

    mTBufferLayouts.resize(program.GetTBuffers().size());
    i = 0;
    for (auto const& tb : program.GetTBuffers())
    {
        mData[TextureBuffer::shaderDataLookup].push_back(
            Data(GT_TEXTURE_BUFFER, tb.GetName(), tb.GetBindPoint(),
                tb.GetNumBytes(), 0, false));

        tb.GenerateLayout(mTBufferLayouts[i]);
        ++i;
    }

    for (auto const& sb : program.GetSBuffers())
    {
        unsigned int ctrtype = 0xFFFFFFFFu;
        switch (sb.GetType())
        {
        case HLSLStructuredBuffer::SBT_BASIC:
            ctrtype = StructuredBuffer::CT_NONE;
            break;

        case HLSLStructuredBuffer::SBT_APPEND:
        case HLSLStructuredBuffer::SBT_CONSUME:
            ctrtype = StructuredBuffer::CT_APPEND_CONSUME;
            break;

        case HLSLStructuredBuffer::SBT_COUNTER:
            ctrtype = StructuredBuffer::CT_COUNTER;
            break;

        default:
            LogError("Unexpected structured buffer option: " +
                std::to_string(static_cast<int>(sb.GetType())));
        }

        mData[StructuredBuffer::shaderDataLookup].push_back(
            Data(GT_STRUCTURED_BUFFER, sb.GetName(), sb.GetBindPoint(),
                sb.GetNumBytes(), ctrtype, sb.IsGpuWritable()));
    }

    for (auto const& rb : program.GetRBuffers())
    {
        mData[RawBuffer::shaderDataLookup].push_back(
            Data(GT_RAW_BUFFER, rb.GetName(), rb.GetBindPoint(),
                rb.GetNumBytes(), 0, rb.IsGpuWritable()));
    }

    for (auto const& tx : program.GetTextures())
    {
        mData[TextureSingle::shaderDataLookup].push_back(
            Data(GT_TEXTURE_SINGLE, tx.GetName(), tx.GetBindPoint(), 0,
                tx.GetNumDimensions(), tx.IsGpuWritable()));
    }

    for (auto const& ta : program.GetTextureArrays())
    {
        mData[TextureArray::shaderDataLookup].push_back(
            Data(GT_TEXTURE_ARRAY, ta.GetName(), ta.GetBindPoint(), 0,
                ta.GetNumDimensions(), ta.IsGpuWritable()));
    }

    for (auto const& s : program.GetSamplerStates())
    {
        mData[SamplerState::shaderDataLookup].push_back(
            Data(GT_SAMPLER_STATE, s.GetName(), s.GetBindPoint(), 0, 0,
                false));
    }

    // The conversion depends on the HLSLShader::Type ordering to be the
    // same as GraphicsObjectType for GL_SHADER through GL_COMPUTE_SHADER.
    int index = program.GetShaderTypeIndex();
    mType = static_cast<GraphicsObjectType>(GT_SHADER + 1 + index);
}
#endif

int Shader::Get(std::string const& name) const
{
    for (int lookup = 0; lookup < NUM_LOOKUP_INDICES; ++lookup)
    {
        int handle = 0;
        for (auto const& data : mData[lookup])
        {
            if (name == data.name)
            {
                return handle;
            }
            ++handle;
        }
    }
    return -1;
}

unsigned int Shader::GetConstantBufferSize(int handle) const
{
    auto const& data = mData[ConstantBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        return data[handle].numBytes;
    }

    LogError("Invalid handle for object.");
    return 0;
}

unsigned int Shader::GetConstantBufferSize(std::string const& name) const
{
    int handle = 0;
    for (auto& data : mData[ConstantBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            return data.numBytes;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return 0;
}

unsigned int Shader::GetTextureBufferSize(int handle) const
{
    auto const& data = mData[TextureBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        return data[handle].numBytes;
    }

    LogError("Invalid handle for object.");
    return 0;
}

unsigned int Shader::GetTextureBufferSize(std::string const& name) const
{
    int handle = 0;
    for (auto& data : mData[TextureBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            return data.numBytes;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return 0;
}

unsigned int Shader::GetStructuredBufferSize(int handle) const
{
    auto const& data = mData[StructuredBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        return data[handle].numBytes;
    }

    LogError("Invalid handle for object.");
    return 0;
}

unsigned int Shader::GetStructuredBufferSize(std::string const& name) const
{
    int handle = 0;
    for (auto& data : mData[StructuredBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            return data.numBytes;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return 0;
}

bool Shader::GetConstantBufferLayout(int handle, BufferLayout& layout) const
{
    auto const& data = mData[ConstantBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        layout = mCBufferLayouts[handle];
        return true;
    }

    LogError("Invalid handle for object.");
    return false;
}

bool Shader::GetConstantBufferLayout(std::string const& name, BufferLayout& layout) const
{
    int handle = 0;
    for (auto& data : mData[ConstantBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            layout = mCBufferLayouts[handle];
            return true;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return false;
}

bool Shader::GetTextureBufferLayout(int handle, BufferLayout& layout) const
{
    auto const& data = mData[TextureBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        layout = mTBufferLayouts[handle];
        return true;
    }

    LogError("Invalid handle for object.");
    return false;
}

bool Shader::GetTextureBufferLayout(std::string const& name, BufferLayout& layout) const
{
    int handle = 0;
    for (auto& data : mData[TextureBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            layout = mTBufferLayouts[handle];
            return true;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return false;
}

#if defined(GTE_DEV_OPENGL)
bool Shader::GetStructuredBufferLayout(int handle, BufferLayout& layout) const
{
    auto const& data = mData[StructuredBuffer::shaderDataLookup];
    if (0 <= handle && handle < static_cast<int>(data.size()))
    {
        layout = mSBufferLayouts[handle];
        return true;
    }

    LogError("Invalid handle for object.");
    return false;
}

bool Shader::GetStructuredBufferLayout(std::string const& name, BufferLayout& layout) const
{
    int handle = 0;
    for (auto& data : mData[StructuredBuffer::shaderDataLookup])
    {
        if (name == data.name)
        {
            layout = mSBufferLayouts[handle];
            return true;
        }
        ++handle;
    }

    LogError("Cannot find object " + name + ".");
    return false;
}
#endif

bool Shader::IsValid(Data const& goal, ConstantBuffer* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_CONSTANT_BUFFER)
    {
        LogError("Mismatch of buffer type.");
        return false;
    }

    if (resource->GetNumBytes() >= static_cast<size_t>(goal.numBytes))
    {
        return true;
    }

    LogError("Invalid number of bytes.");
    return false;
}

bool Shader::IsValid(Data const& goal, TextureBuffer* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_TEXTURE_BUFFER)
    {
        LogError("Mismatch of buffer type.");
        return false;
    }

    if (resource->GetNumBytes() >= static_cast<size_t>(goal.numBytes))
    {
        return true;
    }

    LogError("Invalid number of bytes.");
    return false;
}

bool Shader::IsValid(Data const& goal, StructuredBuffer* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_STRUCTURED_BUFFER)
    {
        LogError("Mismatch of buffer type.");
        return false;
    }

#if !defined(GTE_DEV_OPENGL)
    // GL4 reflection does not provide information about writable access of
    // buffer objects in a shader because by definition, shader storage buffer
    // objects can be read-write by shaders.  For GL4, the isGpuWritable flag is
    // used to indicate whether the structured buffer has a counter attached or not.
    if (goal.isGpuWritable && resource->GetUsage() != Resource::SHADER_OUTPUT)
    {
        LogError("Mismatch of GPU write flag.");
        return false;
    }
#endif

#if defined(GTE_DEV_OPENGL)
    // OpenGL does not have the concept of an append-consume type structured
    // buffer nor does it have the concept of a structured buffer with counter.
    // But, this GL4 support does associate an atomic counter with a structured
    // buffer as long as it has the same name.  If the shader is expecting
    // a counter, then the structured buffer needs to be declared with one.
    if (goal.isGpuWritable && (StructuredBuffer::CT_NONE == resource->GetCounterType()))
    {
        LogError("Mismatch of counter type.");
        return false;
    }
#else
    // A countered structure buffer can be attached as a read-only input to
    // a shader.  We care about the mismatch in counter type only when the
    // shader needs a countered structure buffer but the attached resource
    // does not have one.
    if (goal.extra != 0 
    &&  goal.extra != static_cast<unsigned int>(resource->GetCounterType()))
    {
        LogError("Mismatch of counter type.");
        return false;
    }
#endif

    return true;
}

bool Shader::IsValid(Data const& goal, RawBuffer* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_RAW_BUFFER)
    {
        LogError("Mismatch of buffer type.");
        return false;
    }

    if (goal.isGpuWritable && resource->GetUsage() != Resource::SHADER_OUTPUT)
    {
        LogError("Mismatch of GPU write flag.");
        return false;
    }

    return true;
}

bool Shader::IsValid(Data const& goal, TextureSingle* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_TEXTURE_SINGLE)
    {
        LogError("Mismatch of texture type.");
        return false;
    }

#if !defined(GTE_DEV_OPENGL)
    // GL4 reflection does not provide information about writable access
    // of gimage* and gsampler* objects in a shader.  For GL4, the isGpuWritable flag is
    // used to indicate whether the texture could be writable in shader which
    // is false for gshader* objects and is true for gimage* objects.
    if (goal.isGpuWritable && resource->GetUsage() != Resource::SHADER_OUTPUT)
    {
        LogError("Mismatch of GPU write flag.");
        return false;
    }
#endif

    if (goal.extra != resource->GetNumDimensions())
    {
        LogError("Mismatch of texture dimensions.");
        return false;
    }

    // TODO: Add validation for HLSLTexture::Component and number of
    // components (requires comparison to TextureFormat value).
    return true;
}

bool Shader::IsValid(Data const& goal, TextureArray* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_TEXTURE_ARRAY)
    {
        LogError("Mismatch of texture type.");
        return false;
    }

#if !defined(GTE_DEV_OPENGL)
    // GL4 reflection does not provide information about writable access
    // of gimage* and gsampler* objects in a shader.  For GL4, the isGpuWritable flag is
    // used to indicate whether the texture could be writable in shader which
    // is false for gshader* objects and is true for gimage* objects.
    if (goal.isGpuWritable && resource->GetUsage() != Resource::SHADER_OUTPUT)
    {
        LogError("Mismatch of GPU write flag.");
        return false;
    }
#endif

    if (goal.extra != resource->GetNumDimensions())
    {
        LogError("Mismatch of texture dimensions.");
        return false;
    }

    // TODO: Add validation for HLSLTexture::Component and number of
    // components (requires comparison to TextureFormat value).
    return true;
}

bool Shader::IsValid(Data const& goal, SamplerState* resource) const
{
    if (!resource)
    {
        LogError("Resource is null.");
        return false;
    }

    if (goal.type != GT_SAMPLER_STATE)
    {
        LogError("Mismatch of state.");
        return false;
    }

    return true;
}

