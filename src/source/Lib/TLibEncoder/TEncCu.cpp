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
  m_cuChromaQpOffsetIdxPlus1       = 0;
  m_bFastDeltaQP                   = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
}

Void TEncCu::destroy()
{
  Int i;

  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }
    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }
    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }
    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }
    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }
    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }
    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }
    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }
    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }
  }
  if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }
  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }

  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }
  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }
  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }
  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }
  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }
  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }
  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant();
  m_pcRdCost           = pcEncTop->getRdCost();

  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
  m_pcBinCABAC         = pcEncTop->getBinCABAC();

  m_pppcRDSbacCoder    = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder  = pcEncTop->getRDGoOnSbacCoder();

  m_pcRateCtrl         = pcEncTop->getRateCtrl();
  m_lumaQPOffset       = 0;
  initLumaDeltaQpLUT();
#if JVET_V0078
  m_smoothQPoffset     = 0;
#endif
#if JVET_Y0077_BIM
  m_BimQPoffset        = 0;
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** 
 \param  pCtu pointer of CU data class
 */
Void TEncCu::compressCtu( TComDataCU* pCtu )
{
  // initialize CU data
  m_ppcBestCU[0]->initCtu( pCtu->getPic(), pCtu->getCtuRsAddr() );
  m_ppcTempCU[0]->initCtu( pCtu->getPic(), pCtu->getCtuRsAddr() );
  m_bEncodeDQP         = false;

  // analysis of CU
  DEBUG_STRING_NEW(sDebug)

  xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );
  DEBUG_STRING_OUTPUT(std::cout, sDebug)

#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(pCtu->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xCtuCollectARLStats( pCtu );
    }
  }
#endif
}
/** \param  pCtu  pointer of CU data class
 */
Void TEncCu::encodeCtu ( TComDataCU* pCtu )
{
  if ( pCtu->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  if ( pCtu->getSlice()->getUseChromaQpAdj() )
  {
    setCodeChromaQpAdjFlag(true);
  }

  // Encode CU data
  xEncodeCU( pCtu, 0, 0 );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void TEncCu::initLumaDeltaQpLUT()
{
  const LumaLevelToDeltaQPMapping &mapping=m_pcEncCfg->getLumaLevelToDeltaQPMapping();

  if ( !mapping.isEnabled() )
  {
    return;
  }

  // map the sparse LumaLevelToDeltaQPMapping.mapping to a fully populated linear table.

  Int         lastDeltaQPValue=0;
  std::size_t nextSparseIndex=0;
  for(Int index=0; index<LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; index++)
  {
    while (nextSparseIndex < mapping.mapping.size() && index>=mapping.mapping[nextSparseIndex].first)
    {
      lastDeltaQPValue=mapping.mapping[nextSparseIndex].second;
      nextSparseIndex++;
    }
    m_lumaLevelToDeltaQPLUT[index]=lastDeltaQPValue;
  }
}

Int TEncCu::calculateLumaDQP(TComDataCU *pCU, const UInt absPartIdx, const TComYuv * pOrgYuv)
{
  const Pel *pY = pOrgYuv->getAddr(COMPONENT_Y, absPartIdx);
  const Int stride  = pOrgYuv->getStride(COMPONENT_Y);
  Int width = pCU->getWidth(absPartIdx);
  Int height = pCU->getHeight(absPartIdx);
  Double avg = 0;

  // limit the block by picture size
  const TComSPS* pSPS = pCU->getSlice()->getSPS();
  if ( pCU->getCUPelX() + width > pSPS->getPicWidthInLumaSamples() )
  {
    width = pSPS->getPicWidthInLumaSamples() - pCU->getCUPelX();
  }
  if ( pCU->getCUPelY() + height > pSPS->getPicHeightInLumaSamples() )
  {
    height = pSPS->getPicHeightInLumaSamples() - pCU->getCUPelY();
  }

  // Get QP offset derived from Luma level
  if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_AVG_METHOD )
  {
    // Use avg method
    Int sum = 0;
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        sum += pY[x];
      }
      pY += stride;
    }
    avg = (Double)sum/(width*height);
  }
  else
  {
    // Use maximum luma value
    Int maxVal = 0;
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        if (pY[x] > maxVal)
        {
          maxVal = pY[x];
        }
      }
      pY += stride;
    }
    // use a percentage of the maxVal
    avg = (Double)maxVal * m_pcEncCfg->getLumaLevelToDeltaQPMapping().maxMethodWeight;
  }

  Int lumaIdx = Clip3<Int>(0, Int(LUMA_LEVEL_TO_DQP_LUT_MAXSIZE)-1, Int(avg+0.5) );
  Int QP = m_lumaLevelToDeltaQPLUT[lumaIdx];
  return QP;
}

#if JVET_V0078
Int TEncCu::calculateLumaDQPsmooth(TComDataCU *pCU, const UInt absPartIdx, const TComYuv * pOrgYuv, Int iBaseQP)
{
  const Pel *pY = pOrgYuv->getAddr(COMPONENT_Y, absPartIdx);
  const Int stride = pOrgYuv->getStride(COMPONENT_Y);
  Double avg = 0;
  Double diff = 0;
  Int width = pCU->getWidth(absPartIdx);
  Int height = pCU->getHeight(absPartIdx);
  Double thr = (Double)m_pcEncCfg->getSmoothQPReductionThreshold()*height*width;
  Int iQP = 0;
  if (height == 64 && width == 64)
  {
    Int sum = 0;
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        sum += pY[x];
      }
      pY += stride;
    }
    avg = (Double)sum;

    // determine parameters for 1+x+y+x*x+y*y model
    const Int numBasis = 6;
    Double invb[numBasis][numBasis] = { {0.001*0.244140625000000,                   0,                   0,                   0,                   0,                   0},
                                        {0,   0.001*0.013204564833946,   0.001*0.002080251479290, -0.001*0.000066039729501, -0.001*0.000165220364313,   0.000000000000000},
                                        {0,   0.001*0.002080251479290,   0.001*0.013204564833946, -0.001*0.000066039729501,   0.000000000000000, -0.001*0.000165220364313},
                                        {0, -0.001*0.000066039729501, -0.001*0.000066039729501,   0.001*0.000002096499349, 0.000000000000000, 0.000000000000000},
                                        {0, -0.001*0.000165220364313,   0.000000000000000, 0.000000000000000,   0.001*0.000002622545465, 0.000000000000000},
                                        {0,   0.000000000000000, -0.001*0.000165220364313, 0.000000000000000, 0.000000000000000,   0.001*0.000002622545465} };
    Double boffset[5] = { -31.5, -31.5, -992.25, -1333.5, -1333.5 };

    Double b1sum = avg;
    Double b2sum = 0.0;
    Double b3sum = 0.0;
    Double b4sum = 0.0;
    Double b5sum = 0.0;;
    Double b6sum = 0.0;;

    const Pel *pY1 = pOrgYuv->getAddr(COMPONENT_Y, absPartIdx);
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        b2sum += ((Double)pY1[x])*((Double)x + boffset[0]);
        b3sum += ((Double)pY1[x])*((Double)y + boffset[1]);
        b4sum += ((Double)pY1[x])*((Double)x*(Double)y + boffset[2]);
        b5sum += ((Double)pY1[x])*((Double)x*(Double)x + boffset[3]);
        b6sum += ((Double)pY1[x])*((Double)y*(Double)y + boffset[4]);
      }
      pY1 += stride;
    }

    Double r[numBasis];
    for (Int b = 0; b < numBasis; b++)
    {
      r[b] = invb[b][0] * b1sum + invb[b][1] * b2sum + invb[b][2] * b3sum + invb[b][3] * b4sum + invb[b][4] * b5sum + invb[b][5] * b6sum;
    }

    // compute SAD for model
    const Pel *pY2 = pOrgYuv->getAddr(COMPONENT_Y, absPartIdx);
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        diff += abs((Int)pY2[x] - (Int)(r[0] + r[1] * ((Double)x + boffset[0]) + r[2] * ((Double)y + boffset[1]) + r[3] * ((Double)x*(Double)y + boffset[2]) + r[4] * ((Double)x*(Double)x + boffset[3]) + r[5] * ((Double)y*(Double)y + boffset[4])));
      }
      pY2 += stride;
    }
    if (diff < thr)
    {
      iQP = std::max(m_pcEncCfg->getSmoothQPReductionLimit(), std::min(0, (Int)(m_pcEncCfg->getSmoothQPReductionModelScale()*(Double)iBaseQP + m_pcEncCfg->getSmoothQPReductionModelOffset())));
    }
  }
  return iQP;
}
#endif

//! Derive small set of test modes for AMP encoder speed-up
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
  if ( pcBestCU->getPartitionSize(0) == SIZE_2NxN )
  {
    bTestAMP_Hor = true;
  }
  else if ( pcBestCU->getPartitionSize(0) == SIZE_Nx2N )
  {
    bTestAMP_Ver = true;
  }
  else if ( pcBestCU->getPartitionSize(0) == SIZE_2Nx2N && pcBestCU->getMergeFlag(0) == false && pcBestCU->isSkipped(0) == false )
  {
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  {
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( eParentPartSize == NUMBER_OF_PART_SIZES ) //! if parent is intra
  {
    if ( pcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
      bTestMergeAMP_Hor = true;
    }
    else if ( pcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;
    }
  }

  if ( pcBestCU->getPartitionSize(0) == SIZE_2Nx2N && pcBestCU->isSkipped(0) == false )
  {
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( pcBestCU->getWidth(0) == 64 )
  {
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }
#else
  //! Utilizing the partition size of parent PU
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  {
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  {
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }
#endif
}
#endif


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Compress a CU block recursively with enabling sub-CTU-level delta QP
 *  - for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth )
#endif
{
  TComPic* pcPic = rpcBestCU->getPic();
  DEBUG_STRING_NEW(sDebug)
  const TComPPS &pps=*(rpcTempCU->getSlice()->getPPS());
  const TComSPS &sps=*(rpcTempCU->getSlice()->getSPS());
  
  // These are only used if getFastDeltaQp() is true
  const UInt fastDeltaQPCuMaxSize    = Clip3(sps.getMaxCUHeight()>>sps.getLog2DiffMaxMinCodingBlockSize(), sps.getMaxCUHeight(), 32u);

  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getCtuRsAddr(), rpcBestCU->getZorderIdxInCtu() );

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool    earlyDetectionSkipMode = false;

  const UInt uiLPelX   = rpcBestCU->getCUPelX();
  const UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
  const UInt uiTPelY   = rpcBestCU->getCUPelY();
  const UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;
  const UInt uiWidth   = rpcBestCU->getWidth(0);

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;

  const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();

  if( uiDepth <= pps.getMaxCuDQPDepth() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }

  if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    if ( uiDepth <= pps.getMaxCuDQPDepth() )
    {
      // keep using the same m_QP_LUMA_OFFSET in the same CTU
      m_lumaQPOffset = calculateLumaDQP(rpcTempCU, 0, m_ppcOrigYuv[uiDepth]);
    }
    iMinQP = Clip3(-sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP - m_lumaQPOffset);
    iMaxQP = iMinQP; // force encode choose the modified QO
  }

#if JVET_V0078
  if (m_pcEncCfg->getSmoothQPReductionEnable())
  {
    if (uiDepth <= pps.getMaxCuDQPDepth())
    {
      m_smoothQPoffset = 0;

      // enable smooth QP reduction on selected frames
      bool checkSmoothQP = false;
      if (m_pcEncCfg->getSmoothQPReductionPeriodicity() != 0)
      {
        checkSmoothQP = ((m_pcEncCfg->getSmoothQPReductionPeriodicity() == 0) && rpcTempCU->getSlice()->isIntra()) || (m_pcEncCfg->getSmoothQPReductionPeriodicity() == 1) || ((rpcTempCU->getSlice()->getPOC() % m_pcEncCfg->getSmoothQPReductionPeriodicity()) == 0);
      }
      else
      {
        checkSmoothQP = ((m_pcEncCfg->getSmoothQPReductionPeriodicity() == 0) && rpcTempCU->getSlice()->isIntra());
      }
      if (checkSmoothQP)
      {
        if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
        {
          m_smoothQPoffset = calculateLumaDQPsmooth(rpcTempCU, 0, m_ppcOrigYuv[uiDepth], iBaseQP - m_lumaQPOffset);
        }
        else
        {
          m_smoothQPoffset = calculateLumaDQPsmooth(rpcTempCU, 0, m_ppcOrigYuv[uiDepth], iBaseQP);
        }
      }
    }
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      iMinQP = Clip3(-sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP - m_lumaQPOffset + m_smoothQPoffset);
    }
    else
    {
      iMinQP = Clip3(-sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP + m_smoothQPoffset);
    }
    iMaxQP = iMinQP; // force encode choose the modified QO
  }
#endif

#if JVET_Y0077_BIM
  if( m_pcEncCfg->getBIM() )
  {
    if ( uiDepth <= pps.getMaxCuDQPDepth() )
    {
      std::map<Int, Int*> *QPmap = m_pcEncCfg->getAdaptQPmap();
      Int iCTUoffset = uiTPelY / MAX_CU_SIZE * pcPic->getFrameWidthInCtus() + uiLPelX / MAX_CU_SIZE;
      m_BimQPoffset = ( QPmap->find( pcPic->getPOC() ) == QPmap->end() ) ? 0 : (*QPmap)[pcPic->getPOC()][iCTUoffset];
    }
    
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP - idQP - m_lumaQPOffset + m_smoothQPoffset + m_BimQPoffset );
    iMaxQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP + idQP - m_lumaQPOffset + m_smoothQPoffset + m_BimQPoffset );
  }
#endif

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  // transquant-bypass (TQB) processing loop variable initialisation ---

  const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

  if ( (pps.getTransquantBypassEnabledFlag()) )
  {
    isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
    iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      iMaxQP = iMinQP;
    }
  }

  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());

  const Bool bBoundary = !( uiRPelX < sps.getPicWidthInLumaSamples() && uiBPelY < sps.getPicHeightInLumaSamples() );

  if ( !bBoundary )
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
      const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

      if (bIsLosslessMode)
      {
        iQP = lowestQP;
      }
#if JVET_Y0077_BIM
      if ((m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || m_pcEncCfg->getSmoothQPReductionEnable() || m_pcEncCfg->getBIM()) && uiDepth <= pps.getMaxCuDQPDepth())
#else
#if JVET_V0078
	  if ((m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || m_pcEncCfg->getSmoothQPReductionEnable()) && uiDepth <= pps.getMaxCuDQPDepth())
#else
	  if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && uiDepth <= pps.getMaxCuDQPDepth() )
#endif
#endif
      {
        getSliceEncoder()->updateLambda(pcSlice, iQP);
      }

      m_cuChromaQpOffsetIdxPlus1 = 0;
      if (pcSlice->getUseChromaQpAdj())
      {
        /* Pre-estimation of chroma QP based on input block activity may be performed
         * here, using for example m_ppcOrigYuv[uiDepth] */
        /* To exercise the current code, the index used for adjustment is based on
         * block position
         */
        Int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
                          std::max<Int>(0, sps.getLog2DiffMaxMinCodingBlockSize()-Int(pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth()));
        m_cuChromaQpOffsetIdxPlus1 = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1);
      }

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
      {
        iQP = iMinQP;
      }
    }

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP); // If lossless, then iQP is irrelevant for subsequent modules.

        if (bIsLosslessMode)
        {
          iQP = lowestQP;
        }

        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN

          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
          {
            if( uiDepth == sps.getLog2DiffMaxMinCodingBlockSize() && doNotBlockPu)
            {
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }

          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if(sps.getUseAMP() && uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() )
          {
#if AMP_ENC_SPEEDUP
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
          }
        }

        // do normal intra modes
        // speedup for inter frames
#if MCTS_ENC_CHECK
        if ( m_pcEncCfg->getTMCTSSEITileConstraint() || (rpcBestCU->getSlice()->getSliceType() == I_SLICE) ||
             ((!m_pcEncCfg->getDisableIntraPUsInInterSlices()) && (
             (rpcBestCU->getCbf(0, COMPONENT_Y) != 0) ||
             ((rpcBestCU->getCbf(0, COMPONENT_Cb) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
             ((rpcBestCU->getCbf(0, COMPONENT_Cr) != 0) && (numberValidComponents > COMPONENT_Cr))  // avoid very complex intra if it is unlikely
            )))
        {
#else
        if((rpcBestCU->getSlice()->getSliceType() == I_SLICE)                                        ||
            ((!m_pcEncCfg->getDisableIntraPUsInInterSlices()) && (
              (rpcBestCU->getCbf( 0, COMPONENT_Y  ) != 0)                                            ||
             ((rpcBestCU->getCbf( 0, COMPONENT_Cb ) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
             ((rpcBestCU->getCbf( 0, COMPONENT_Cr ) != 0) && (numberValidComponents > COMPONENT_Cr))  // avoid very complex intra if it is unlikely
            )))
        {
#endif 
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if( uiDepth == sps.getLog2DiffMaxMinCodingBlockSize() )
          {
            if( rpcTempCU->getWidth(0) > ( 1 << sps.getQuadtreeTULog2MinSize() ) )
            {
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }
        }

        // test PCM
        if(sps.getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<sps.getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<sps.getPCMLog2MinSize()) )
        {
          UInt uiRawBits = getTotalBits(rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), rpcBestCU->getPic()->getChromaFormat(), sps.getBitDepths().recon);
          UInt uiBestBits = rpcBestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          }
        }

        if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
        {
          iQP = iMinQP;
        }
      }
    }

    if( rpcBestCU->getTotalCost()!=MAX_DOUBLE )
    {
      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
      rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
      rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
      m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
    }
  }

  // copy original YUV samples to PCM buffer
  if( rpcBestCU->getTotalCost()!=MAX_DOUBLE && rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
  }

  if( uiDepth == pps.getMaxCuDQPDepth() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
#if JVET_Y0077_BIM
    if (m_pcEncCfg->getBIM())
    {
      std::map<Int, Int*> *QPmap = m_pcEncCfg->getAdaptQPmap();
      Int iCTUoffset = uiTPelY / MAX_CU_SIZE * pcPic->getFrameWidthInCtus() + uiLPelX / MAX_CU_SIZE;
      Int iOffset = ( QPmap->find( pcPic->getPOC() ) == QPmap->end() ) ? 0 : (*QPmap)[pcPic->getPOC()][iCTUoffset];
      iMinQP += iOffset;
      iMaxQP += iOffset;
    }
#endif
  }
  else if( uiDepth < pps.getMaxCuDQPDepth() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    const Int iStartQP = rpcTempCU->getQP(0);
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }

  if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    iMinQP = Clip3(-sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP - m_lumaQPOffset);
    iMaxQP = iMinQP;
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
  {
    iMaxQP = iMinQP; // If all TUs are forced into using transquant bypass, do not loop here.
  }
