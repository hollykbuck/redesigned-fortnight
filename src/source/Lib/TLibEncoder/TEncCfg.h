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

/** \file     TEncCfg.h
    \brief    encoder configuration class (header)
*/

#ifndef __TENCCFG__
#define __TENCCFG__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComSlice.h"
#if JVET_T0050_ANNOTATED_REGIONS_SEI
#include "TLibCommon/SEI.h"
#endif
#include <assert.h>

struct GOPEntry
{
  Int m_POC;
  Int m_QPOffset;
  Double m_QPOffsetModelOffset;
  Double m_QPOffsetModelScale;
  Int m_CbQPoffset;
  Int m_CrQPoffset;
  Double m_QPFactor;
  Int m_tcOffsetDiv2;
  Int m_betaOffsetDiv2;
  Int m_temporalId;
  Bool m_refPic;
  Int m_numRefPicsActive;
  SChar m_sliceType;
  Int m_numRefPics;
  Int m_referencePics[MAX_NUM_REF_PICS];
  Int m_usedByCurrPic[MAX_NUM_REF_PICS];
  Int m_interRPSPrediction;
  Int m_deltaRPS;
  Int m_numRefIdc;
  Int m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_isEncoded;
  GOPEntry()
  : m_POC(-1)
  , m_QPOffset(0)
  , m_QPOffsetModelOffset(0)
  , m_QPOffsetModelScale(0)
  , m_CbQPoffset(0)
  , m_CrQPoffset(0)
  , m_QPFactor(0)
  , m_tcOffsetDiv2(0)
  , m_betaOffsetDiv2(0)
  , m_temporalId(0)
  , m_refPic(false)
  , m_numRefPicsActive(0)
  , m_sliceType('P')
  , m_numRefPics(0)
  , m_interRPSPrediction(false)
  , m_deltaRPS(0)
  , m_numRefIdc(0)
  , m_isEncoded(false)
  {
    ::memset( m_referencePics, 0, sizeof(m_referencePics) );
    ::memset( m_usedByCurrPic, 0, sizeof(m_usedByCurrPic) );
    ::memset( m_refIdc,        0, sizeof(m_refIdc) );
  }
};

