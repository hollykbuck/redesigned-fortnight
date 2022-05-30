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

/** \file     TDecConformance.cpp
    \brief    Decoder conformance functions
*/

#include "TDecConformance.h"
#include "TLibCommon/TComSlice.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicSym.h"
#include "NALread.h"
#include <math.h>

#if !DPB_ENCODER_USAGE_CHECK
UInt
LevelTierFeatures::getMaxPicWidthInLumaSamples()  const
{
  return UInt(sqrt(maxLumaPs*8.0));
}

UInt
LevelTierFeatures::getMaxPicHeightInLumaSamples() const
{
  return UInt(sqrt(maxLumaPs*8.0));
}
#endif

UInt
TDecConformanceCheck::getMinLog2CtbSize(const TComPTL &ptl,
                                        UInt layerPlus1)
{
  const ProfileTierLevel *pPTL = (layerPlus1==0) ? ptl.getGeneralPTL() : ptl.getSubLayerPTL(layerPlus1-1);
  return (pPTL->getLevelIdc() < Level::LEVEL5) ? 4 : 5;
}


UInt
TDecConformanceCheck::getMaxLog2CtbSize(const TComPTL &/*ptl*/,
                                        UInt /*layerPlus1*/)
{
  return MAX_CU_DEPTH;
}


TDecConformanceCheck::TDecConformanceCheck()
#if MCTS_ENC_CHECK
  :m_tmctsCheckEnabled(false)
#if DECODER_PARTIAL_CONFORMANCE_CHECK
  , m_numberOfSlicesInPicture(0)
  , m_bytesInPicture(0)
#endif
#else
#if DECODER_PARTIAL_CONFORMANCE_CHECK
  : m_numberOfSlicesInPicture(0)
  , m_bytesInPicture(0)
#endif
#endif
{
}

#if DECODER_PARTIAL_CONFORMANCE_CHECK != 0

#if !DPB_ENCODER_USAGE_CHECK
static const UInt64 MAX_CNFUINT64 = std::numeric_limits<UInt64>::max();

static const LevelTierFeatures mainLevelTierInfo[] =
{
    { Level::LEVEL1  ,    36864, {      350,        0 },       16,        1,        1,     552960ULL, {     128,        0 }, { 2, 2} },
    { Level::LEVEL2  ,   122880, {     1500,        0 },       16,        1,        1,    3686400ULL, {    1500,        0 }, { 2, 2} },
