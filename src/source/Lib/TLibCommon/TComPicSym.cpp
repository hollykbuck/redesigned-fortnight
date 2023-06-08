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

/** \file     TComPicSym.cpp
    \brief    picture symbol class
*/

#include "TComPicSym.h"
#include "TComSampleAdaptiveOffset.h"
#include "TComSlice.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPicSym::TComPicSym()
:m_frameWidthInCtus(0)
,m_frameHeightInCtus(0)
,m_uiMinCUWidth(0)
,m_uiMinCUHeight(0)
,m_uhTotalDepth(0)
,m_numPartitionsInCtu(0)
,m_numPartInCtuWidth(0)
,m_numPartInCtuHeight(0)
,m_numCtusInFrame(0)
,m_apSlices()
,m_pictureCtuArray(NULL)
,m_numTileColumnsMinus1(0)
,m_numTileRowsMinus1(0)
,m_ctuTsToRsAddrMap(NULL)
,m_puiTileIdxMap(NULL)
,m_ctuRsToTsAddrMap(NULL)
#if REDUCED_ENCODER_MEMORY
,m_dpbPerCtuData(NULL)
#endif
,m_saoBlkParams(NULL)
#if ADAPTIVE_QP_SELECTION
,m_pParentARLBuffer(NULL)
#endif
{}


TComPicSym::~TComPicSym()
{
  destroy();
}


#if REDUCED_ENCODER_MEMORY
Void TComPicSym::create  ( const TComSPS &sps, const TComPPS &pps, UInt uiMaxDepth, const Bool bAllocateCtuArray )
#else
Void TComPicSym::create  ( const TComSPS &sps, const TComPPS &pps, UInt uiMaxDepth )
#endif
{
  destroy();

  m_sps = sps;
  m_pps = pps;

#if !REDUCED_ENCODER_MEMORY
  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
#endif
  const Int iPicWidth      = sps.getPicWidthInLumaSamples();
  const Int iPicHeight     = sps.getPicHeightInLumaSamples();
  const UInt uiMaxCuWidth  = sps.getMaxCUWidth();
  const UInt uiMaxCuHeight = sps.getMaxCUHeight();

  m_uhTotalDepth       = uiMaxDepth;
  m_numPartitionsInCtu = 1<<(m_uhTotalDepth<<1);

  m_uiMinCUWidth       = uiMaxCuWidth  >> m_uhTotalDepth;
  m_uiMinCUHeight      = uiMaxCuHeight >> m_uhTotalDepth;

  m_numPartInCtuWidth  = uiMaxCuWidth  / m_uiMinCUWidth;  // equivalent to 1<<m_uhTotalDepth
  m_numPartInCtuHeight = uiMaxCuHeight / m_uiMinCUHeight; // equivalent to 1<<m_uhTotalDepth

  m_frameWidthInCtus   = ( iPicWidth %uiMaxCuWidth  ) ? iPicWidth /uiMaxCuWidth  + 1 : iPicWidth /uiMaxCuWidth;
  m_frameHeightInCtus  = ( iPicHeight%uiMaxCuHeight ) ? iPicHeight/uiMaxCuHeight + 1 : iPicHeight/uiMaxCuHeight;

  m_numCtusInFrame     = m_frameWidthInCtus * m_frameHeightInCtus;
#if REDUCED_ENCODER_MEMORY
  m_pictureCtuArray    = NULL;
#else
  m_pictureCtuArray    = new TComDataCU*[m_numCtusInFrame];
#endif

  clearSliceBuffer();
  allocateNewSlice();

#if ADAPTIVE_QP_SELECTION
  if (m_pParentARLBuffer == NULL)
  {
    m_pParentARLBuffer = new TCoeff[uiMaxCuWidth*uiMaxCuHeight*MAX_NUM_COMPONENT];
  }
#endif

#if REDUCED_ENCODER_MEMORY
  if (bAllocateCtuArray)
  {
    prepareForReconstruction();
  }
#else
  for (UInt i=0; i<m_numCtusInFrame ; i++ )
  {
    m_pictureCtuArray[i] = new TComDataCU;
    m_pictureCtuArray[i]->create( chromaFormatIDC, m_numPartitionsInCtu, uiMaxCuWidth, uiMaxCuHeight, false, uiMaxCuWidth >> m_uhTotalDepth
#if ADAPTIVE_QP_SELECTION
      , m_pParentARLBuffer
#endif
      );
  }
#endif

  m_ctuTsToRsAddrMap = new UInt[m_numCtusInFrame+1];
  m_puiTileIdxMap    = new UInt[m_numCtusInFrame];
  m_ctuRsToTsAddrMap = new UInt[m_numCtusInFrame+1];

  for(UInt i=0; i<m_numCtusInFrame; i++ )
  {
    m_ctuTsToRsAddrMap[i] = i;
    m_ctuRsToTsAddrMap[i] = i;
  }

  m_saoBlkParams = new SAOBlkParam[m_numCtusInFrame];


  xInitTiles();
  xInitCtuTsRsAddrMaps();

}

#if REDUCED_ENCODER_MEMORY
Void TComPicSym::prepareForReconstruction()
{
  const ChromaFormat chromaFormatIDC = m_sps.getChromaFormatIdc();
  const UInt uiMaxCuWidth  = m_sps.getMaxCUWidth();
  const UInt uiMaxCuHeight = m_sps.getMaxCUHeight();
  if (m_pictureCtuArray == NULL)
  {
    m_pictureCtuArray = new TComDataCU*[m_numCtusInFrame];

    for (UInt i=0; i<m_numCtusInFrame ; i++ )
    {
      m_pictureCtuArray[i] = new TComDataCU;
      m_pictureCtuArray[i]->create( chromaFormatIDC, m_numPartitionsInCtu, uiMaxCuWidth, uiMaxCuHeight, false, uiMaxCuWidth >> m_uhTotalDepth
#if ADAPTIVE_QP_SELECTION
        , m_pParentARLBuffer
#endif
        );
    }
  }
  if (m_dpbPerCtuData == NULL)
  {
    m_dpbPerCtuData = new DPBPerCtuData[m_numCtusInFrame];
    for(UInt i=0; i<m_numCtusInFrame; i++)
    {
      for(Int j=0; j<NUM_REF_PIC_LIST_01; j++)
      {
        m_dpbPerCtuData[i].m_CUMvField[j].create( m_numPartitionsInCtu );
      }
      m_dpbPerCtuData[i].m_pePredMode = new SChar[m_numPartitionsInCtu];
      memset(m_dpbPerCtuData[i].m_pePredMode, NUMBER_OF_PREDICTION_MODES, m_numPartitionsInCtu);
      m_dpbPerCtuData[i].m_pePartSize = new SChar[m_numPartitionsInCtu];
      memset(m_dpbPerCtuData[i].m_pePartSize, NUMBER_OF_PART_SIZES, m_numPartitionsInCtu);
      m_dpbPerCtuData[i].m_pSlice=NULL;
    }
  }
