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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"
#include "TLibCommon/Debug.h"

#include <cmath>
#include <algorithm>
using namespace std;


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uhTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 \param    chromaFormat  chroma format
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight, ChromaFormat chromaFormat)
{
  Int i;

  m_uhTotalDepth   = uhTotalDepth + 1;
  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];

  m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];

  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;

    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );

    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight, chromaFormat);
  }

  m_bEncodeDQP                     = false;
  m_stillToCodeChromaQpOffsetFlag  = false;
