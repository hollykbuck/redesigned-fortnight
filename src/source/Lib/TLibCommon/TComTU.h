/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __TCOMTU__
#define __TCOMTU__

class TComTU; // forward declaration

#include "CommonDef.h"
#include "TComRectangle.h"
#include "TComChromaFormat.h"

class TComDataCU; // forward declaration

//----------------------------------------------------------------------------------------------------------------------


class TComTU
{
  public:
    typedef enum TU_SPLIT_MODE { DONT_SPLIT=0, VERTICAL_SPLIT=1, QUAD_SPLIT=2, NUMBER_OF_SPLIT_MODES=3 } SPLIT_MODE;

    static const UInt NUMBER_OF_SECTIONS[NUMBER_OF_SPLIT_MODES];

  protected:
    ChromaFormat  mChromaFormat;
    Bool          mbProcessLastOfLevel; // if true, then if size n/2 x n/2 is invalid, the nxn block for a channel is processed only for the last block, not the first.
    UInt          mCuDepth;
    UInt          mTrDepthRelCU[MAX_NUM_COMPONENT];
    UInt          mSection;
    TU_SPLIT_MODE mSplitMode;
    TComRectangle mRect[MAX_NUM_COMPONENT];
    Bool          mCodeAll[MAX_NUM_COMPONENT];
    UInt          mOrigWidth[MAX_NUM_COMPONENT];
    UInt          mOffsets[MAX_NUM_COMPONENT];
    UInt          mAbsPartIdxCU;
    UInt          mAbsPartIdxTURelCU;
    UInt          mAbsPartIdxStep;
    TComDataCU   *mpcCU;
    UInt          mLog2TrLumaSize;
    TComTU       *mpParent;

    TComTU(const TComTU &);           // not defined - do not use
    TComTU&operator=(const TComTU &); // not defined - do not use

  public:
    TComTU(      TComDataCU *pcCU,
           const UInt        absPartIdxCU,
           const UInt        cuDepth,
           const UInt        initTrDepthRelCU);

  protected:
    TComTU(      TComTU        &parentLevel,
           const Bool           bProcessLastOfLevel,
           const TU_SPLIT_MODE  splitMode                 = QUAD_SPLIT,
           const Bool           splitAtCurrentDepth       = false,
           const ComponentID    absPartIdxSourceComponent = COMPONENT_Y
           );

  public:
          TComTU *Parent()       { return mpParent; }
    const TComTU *Parent() const { return mpParent; }

    UInt getCoefficientOffset(const ComponentID compID)        const { return mOffsets[compID]; }

    const TComRectangle &getRect(const ComponentID compID)     const { return mRect[compID];    }

    Bool ProcessingAllQuadrants(const ComponentID compID)      const { return mCodeAll[compID]; }
    Bool ProcessComponentSection(const ComponentID compID)     const { return mRect[compID].width != 0; }
    Bool ProcessChannelSection(const ChannelType chType)       const { return mRect[chType].width != 0; }
