// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2017
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#include <GTEnginePCH.h>
#include <LowLevel/GteLogger.h>
#include <Mathematics/GteBitHacks.h>
#include <Graphics/GteTexture.h>
using namespace gte;


Texture::Texture(unsigned int numItems, DFType format,
    unsigned int numDimensions, unsigned int dim0, unsigned int dim1,
    unsigned int dim2, bool hasMipmaps, bool createStorage)
    :
    Resource(GetTotalElements(numItems, dim0, dim1, dim2, hasMipmaps),
        DataFormat::GetNumBytesPerStruct(format), createStorage),
    mNumItems(numItems),
    mFormat(format),
    mNumDimensions(numDimensions),
    mNumLevels(1),
    mLOffset(numItems),
    mHasMipmaps(hasMipmaps),
    mAutogenerateMipmaps(false)
{
    mType = GT_TEXTURE;

    if (mNumDimensions < 1 || mNumDimensions > 3)
    {
        LogError("Invalid number of dimensions.");
        mNumDimensions = 1;
    }

    // Zero-out all the level information.
    for (unsigned int level = 0; level < MAX_MIPMAP_LEVELS; ++level)
    {
        mLDimension[level][0] = 0;
        mLDimension[level][1] = 0;
        mLDimension[level][2] = 0;
        mLNumBytes[level] = 0;
    }

    for (unsigned int item = 0; item < mNumItems; ++item)
    {
        for (unsigned int level = 0; level < MAX_MIPMAP_LEVELS; ++level)
        {
            mLOffset[item][level] = 0;
        }
    }

    // The base level is always used.
    mLDimension[0][0] = dim0;
    mLDimension[0][1] = dim1;
    mLDimension[0][2] = dim2;
    mLNumBytes[0] = dim0*dim1*dim2*mElementSize;

    if (mHasMipmaps)
    {
        unsigned int log0 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim0));
        unsigned int log1 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim1));
        unsigned int log2 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim2));
        mNumLevels = 1 + std::max(log0, std::max(log1, log2));
        for (unsigned int level = 1; level < mNumLevels; ++level)
        {
            if (dim0 > 1) { dim0 >>= 1; }
            if (dim1 > 1) { dim1 >>= 1; }
            if (dim2 > 1) { dim2 >>= 1; }

            mLNumBytes[level] = dim0*dim1*dim2*mElementSize;
            mLDimension[level][0] = dim0;
            mLDimension[level][1] = dim1;
            mLDimension[level][2] = dim2;
        }

        unsigned int numBytes = 0;
        for (unsigned int item = 0; item < mNumItems; ++item)
        {
            for (unsigned int level = 0; level < mNumLevels; ++level)
            {
                mLOffset[item][level] = numBytes;
                numBytes += mLNumBytes[level];
            }
        }
    }
    else
    {
        for (unsigned int item = 1; item < mNumItems; ++item)
        {
            mLOffset[item][0] = item * mLNumBytes[0];
        }
    }
}

unsigned int Texture::GetIndex(unsigned int item, unsigned int level) const
{
    if (item < mNumItems && level < mNumLevels)
    {
        return mNumLevels * item + level;
    }
    else
    {
        LogError("Invalid input.");
        return 0;
    }
}

Texture::Subresource Texture::GetSubresource(unsigned int index) const
{
    Subresource sr;
    if (index < GetNumSubresources())
    {
        sr.item = index / mNumLevels;
        sr.level = index % mNumLevels;
        sr.data = const_cast<char*>(GetDataFor(sr.item, sr.level));
        sr.rowPitch = mLDimension[sr.level][0] * mElementSize;
        sr.slicePitch = mLDimension[sr.level][1] * sr.rowPitch;
    }
    else
    {
        LogError("Invalid input.");
        sr.item = 0;
        sr.level = 0;
        sr.data = nullptr;
        sr.rowPitch = 0;
        sr.slicePitch = 0;
    }
    return sr;
}

void Texture::AutogenerateMipmaps()
{
    if (mHasMipmaps)
    {
        mAutogenerateMipmaps = true;
        //mUsage = SHADER_OUTPUT;
    }
}

unsigned int Texture::GetTotalElements(unsigned int numItems,
    unsigned int dim0, unsigned int dim1, unsigned int dim2, bool hasMipmaps)
{
    unsigned int numElementsPerItem = dim0 * dim1 * dim2;
    if (hasMipmaps)
    {
        unsigned int log0 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim0));
        unsigned int log1 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim1));
        unsigned int log2 = Log2OfPowerOfTwo(RoundDownToPowerOfTwo(dim2));
        unsigned int numLevels = 1 + std::max(log0, std::max(log1, log2));
        for (unsigned int level = 1; level < numLevels; ++level)
        {
            if (dim0 > 1) { dim0 >>= 1; }
            if (dim1 > 1) { dim1 >>= 1; }
            if (dim2 > 1) { dim2 >>= 1; }

            numElementsPerItem += dim0*dim1*dim2;
        }
    }
    return numItems*numElementsPerItem;
}

