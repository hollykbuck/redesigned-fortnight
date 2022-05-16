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

/** \file     TEncTop.cpp
    \brief    encoder class
*/

#include "TLibCommon/CommonDef.h"
#include "TEncTop.h"
#include "TEncPic.h"
#include "TLibCommon/TComChromaFormat.h"
#if FAST_BIT_EST
#include "TLibCommon/ContextModel.h"
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncTop::TEncTop() :
    m_spsMap(MAX_NUM_SPS)
  , m_ppsMap(MAX_NUM_PPS)
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;
  m_pppcRDSbacCoder   =  NULL;
  m_pppcBinCoderCABAC =  NULL;
  m_cRDGoOnSbacCoder.init( &m_cRDGoOnBinCoderCABAC );
#if ENC_DEC_TRACE
  if (g_hTrace == NULL)
  {
    g_hTrace = fopen( "TraceEnc.txt", "wb" );
  }
  g_bJustDoIt = g_bEncDecTraceDisable;
  g_nSymbolCounter = 0;
#endif

  m_iMaxRefPicNum     = 0;

#if FAST_BIT_EST
  ContextModel::buildNextStateTable();
#endif
}

TEncTop::~TEncTop()
{
#if ENC_DEC_TRACE
  if (g_hTrace != stdout)
  {
    fclose( g_hTrace );
  }
#endif
}

Void TEncTop::create ()
{
  // initialize global variables
  initROM();

  // create processing unit classes
  m_cGOPEncoder.        create( );
  m_cSliceEncoder.      create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth );
  m_cCuEncoder.         create( m_maxTotalCUDepth, m_maxCUWidth, m_maxCUHeight, m_chromaFormatIDC );
  if (m_bUseSAO)
  {
    m_cEncSAO.create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, m_maxTotalCUDepth, m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA], m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
    m_cEncSAO.createEncData(getSaoCtuBoundary());
  }
#if ADAPTIVE_QP_SELECTION
  if (m_bUseAdaptQpSelect)
  {
    m_cTrQuant.initSliceQpDelta();
  }
#endif

  m_cLoopFilter.create( m_maxTotalCUDepth );

  if ( m_RCEnableRateControl )
  {
#if JVET_Y0105_SW_AND_QDF
    m_cRateCtrl.init( m_framesToBeEncoded, m_RCTargetBitrate, (Int)( (Double)m_iFrameRate/m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_uiIntraPeriod, m_iSourceWidth, m_iSourceHeight,
                      m_maxCUWidth, m_maxCUHeight,m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList );
#else
    m_cRateCtrl.init( m_framesToBeEncoded, m_RCTargetBitrate, (Int)( (Double)m_iFrameRate/m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
                      m_maxCUWidth, m_maxCUHeight,m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList );
#endif
  }
  

  m_pppcRDSbacCoder = new TEncSbac** [m_maxTotalCUDepth+1];
#if FAST_BIT_EST
  m_pppcBinCoderCABAC = new TEncBinCABACCounter** [m_maxTotalCUDepth+1];
#else
  m_pppcBinCoderCABAC = new TEncBinCABAC** [m_maxTotalCUDepth+1];
#endif

  for ( Int iDepth = 0; iDepth < m_maxTotalCUDepth+1; iDepth++ )
  {
    m_pppcRDSbacCoder[iDepth] = new TEncSbac* [CI_NUM];
#if FAST_BIT_EST
    m_pppcBinCoderCABAC[iDepth] = new TEncBinCABACCounter* [CI_NUM];
#else
    m_pppcBinCoderCABAC[iDepth] = new TEncBinCABAC* [CI_NUM];
#endif

    for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
    {
      m_pppcRDSbacCoder[iDepth][iCIIdx] = new TEncSbac;
#if FAST_BIT_EST
      m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABACCounter;
#else
      m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABAC;
#endif
      m_pppcRDSbacCoder   [iDepth][iCIIdx]->init( m_pppcBinCoderCABAC [iDepth][iCIIdx] );
    }
  }
}

Void TEncTop::destroy ()
{
  // destroy processing unit classes
  m_cGOPEncoder.        destroy();
  m_cSliceEncoder.      destroy();
  m_cCuEncoder.         destroy();
  m_cEncSAO.            destroyEncData();
  m_cEncSAO.            destroy();
  m_cLoopFilter.        destroy();
  m_cRateCtrl.          destroy();
  m_cSearch.            destroy();
  Int iDepth;
  for ( iDepth = 0; iDepth < m_maxTotalCUDepth+1; iDepth++ )
  {
    for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
    {
      delete m_pppcRDSbacCoder[iDepth][iCIIdx];
      delete m_pppcBinCoderCABAC[iDepth][iCIIdx];
    }
  }

  for ( iDepth = 0; iDepth < m_maxTotalCUDepth+1; iDepth++ )
  {
    delete [] m_pppcRDSbacCoder[iDepth];
    delete [] m_pppcBinCoderCABAC[iDepth];
  }

  delete [] m_pppcRDSbacCoder;
  delete [] m_pppcBinCoderCABAC;

  // destroy ROM
  destroyROM();

  return;
}

Void TEncTop::init(Bool isFieldCoding)
{
  TComSPS &sps0=*(m_spsMap.allocatePS(0)); // NOTE: implementations that use more than 1 SPS need to be aware of activation issues.
  TComPPS &pps0=*(m_ppsMap.allocatePS(0));
  // initialize SPS
  xInitSPS(sps0);
  xInitVPS(m_cVPS, sps0);

  if (m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(sps0.getVuiParameters()->getHrdParameters(), m_iFrameRate, m_RCInitialCpbFullness);
  }
