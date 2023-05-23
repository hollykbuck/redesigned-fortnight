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

/** \file     TComDataCU.cpp
    \brief    CU data structure
    \todo     not all entities are documented
*/

#include "TComDataCU.h"
#include "TComTU.h"
#include "TComPic.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComDataCU::TComDataCU()
{
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_puhDepth           = NULL;

  m_skipFlag           = NULL;

  m_pePartSize         = NULL;
  m_pePredMode         = NULL;
  m_CUTransquantBypass = NULL;
  m_puhWidth           = NULL;
  m_puhHeight          = NULL;
  m_phQP               = NULL;
  m_ChromaQpAdj        = NULL;
  m_pbMergeFlag        = NULL;
  m_puhMergeIndex      = NULL;
  for(UInt i=0; i<MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_puhIntraDir[i]     = NULL;
  }
  m_puhInterDir        = NULL;
  m_puhTrIdx           = NULL;

  for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_puhCbf[comp]                        = NULL;
    m_crossComponentPredictionAlpha[comp] = NULL;
    m_puhTransformSkip[comp]              = NULL;
    m_pcTrCoeff[comp]                     = NULL;
#if ADAPTIVE_QP_SELECTION
    m_pcArlCoeff[comp]                    = NULL;
#endif
    m_pcIPCMSample[comp]                  = NULL;
    m_explicitRdpcmMode[comp]             = NULL;
  }
#if ADAPTIVE_QP_SELECTION
  m_ArlCoeffIsAliasedAllocation = false;
#endif
  m_pbIPCMFlag         = NULL;

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apiMVPIdx[i]       = NULL;
    m_apiMVPNum[i]       = NULL;
  }

  m_bDecSubCu          = false;
}

TComDataCU::~TComDataCU()
{
}

Void TComDataCU::create( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
                        , TCoeff *pParentARLBuffer
#endif
                        )
{
  m_bDecSubCu = bDecSubCu;

  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_uiNumPartition     = uiNumPartition;
  m_unitSize = unitSize;

  if ( !bDecSubCu )
  {
    m_phQP               = (SChar*    )xMalloc(SChar,    uiNumPartition);
    m_puhDepth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhWidth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhHeight          = (UChar*    )xMalloc(UChar,    uiNumPartition);

    m_ChromaQpAdj        = new UChar[ uiNumPartition ];
    m_skipFlag           = new Bool[ uiNumPartition ];
    m_pePartSize         = new SChar[ uiNumPartition ];
    memset( m_pePartSize, NUMBER_OF_PART_SIZES,uiNumPartition * sizeof( *m_pePartSize ) );
    m_pePredMode         = new SChar[ uiNumPartition ];
    m_CUTransquantBypass = new Bool[ uiNumPartition ];

    m_pbMergeFlag        = (Bool*  )xMalloc(Bool,   uiNumPartition);
    m_puhMergeIndex      = (UChar* )xMalloc(UChar,  uiNumPartition);

    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      m_puhIntraDir[ch] = (UChar* )xMalloc(UChar,  uiNumPartition);
    }
    m_puhInterDir        = (UChar* )xMalloc(UChar,  uiNumPartition);

    m_puhTrIdx           = (UChar* )xMalloc(UChar,  uiNumPartition);

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl]       = new SChar[ uiNumPartition ];
      m_apiMVPNum[rpl]       = new SChar[ uiNumPartition ];
      memset( m_apiMVPIdx[rpl], -1,uiNumPartition * sizeof( SChar ) );
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      const UInt chromaShift = getComponentScaleX(compID, chromaFormatIDC) + getComponentScaleY(compID, chromaFormatIDC);
      const UInt totalSize   = (uiWidth * uiHeight) >> chromaShift;

      m_crossComponentPredictionAlpha[compID] = (SChar* )xMalloc(SChar,  uiNumPartition);
      m_puhTransformSkip[compID]              = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_explicitRdpcmMode[compID]             = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_puhCbf[compID]                        = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_pcTrCoeff[compID]                     = (TCoeff*)xMalloc(TCoeff, totalSize);
      memset( m_pcTrCoeff[compID], 0, (totalSize * sizeof( TCoeff )) );

#if ADAPTIVE_QP_SELECTION
      if( pParentARLBuffer != 0 )
      {
        m_pcArlCoeff[compID] = pParentARLBuffer;
        m_ArlCoeffIsAliasedAllocation = true;
        pParentARLBuffer += totalSize;
      }
      else
      {
        m_pcArlCoeff[compID] = (TCoeff*)xMalloc(TCoeff, totalSize);
        m_ArlCoeffIsAliasedAllocation = false;
      }
#endif
      m_pcIPCMSample[compID] = (Pel*   )xMalloc(Pel , totalSize);
    }

    m_pbIPCMFlag         = (Bool*  )xMalloc(Bool, uiNumPartition);

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].create( uiNumPartition );
    }

  }
  else
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].setNumPartition(uiNumPartition );
    }
  }

  // create motion vector fields

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;
}

Void TComDataCU::destroy()
{
  // encoder-side buffer free
  if ( !m_bDecSubCu )
  {
    if ( m_phQP )
    {
      xFree(m_phQP);
      m_phQP = NULL;
    }
    if ( m_puhDepth )
    {
      xFree(m_puhDepth);
      m_puhDepth = NULL;
    }
    if ( m_puhWidth )
    {
      xFree(m_puhWidth);
      m_puhWidth = NULL;
    }
    if ( m_puhHeight )
    {
      xFree(m_puhHeight);
      m_puhHeight = NULL;
    }

    if ( m_skipFlag )
    {
      delete[] m_skipFlag;
      m_skipFlag = NULL;
    }

    if ( m_pePartSize )
    {
      delete[] m_pePartSize;
      m_pePartSize = NULL;
    }
    if ( m_pePredMode )
    {
      delete[] m_pePredMode;
      m_pePredMode = NULL;
    }
    if ( m_ChromaQpAdj )
    {
      delete[] m_ChromaQpAdj;
      m_ChromaQpAdj = NULL;
    }
    if ( m_CUTransquantBypass )
    {
      delete[] m_CUTransquantBypass;
      m_CUTransquantBypass = NULL;
    }
    if ( m_puhInterDir )
    {
      xFree(m_puhInterDir);
      m_puhInterDir = NULL;
    }
    if ( m_pbMergeFlag )
    {
      xFree(m_pbMergeFlag);
      m_pbMergeFlag = NULL;
    }
    if ( m_puhMergeIndex )
    {
      xFree(m_puhMergeIndex);
      m_puhMergeIndex  = NULL;
    }

    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      xFree(m_puhIntraDir[ch]);
      m_puhIntraDir[ch] = NULL;
    }

    if ( m_puhTrIdx )
    {
      xFree(m_puhTrIdx);
      m_puhTrIdx = NULL;
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      if ( m_crossComponentPredictionAlpha[comp] )
      {
        xFree(m_crossComponentPredictionAlpha[comp]);
        m_crossComponentPredictionAlpha[comp] = NULL;
      }
      if ( m_puhTransformSkip[comp] )
      {
        xFree(m_puhTransformSkip[comp]);
        m_puhTransformSkip[comp] = NULL;
      }
      if ( m_puhCbf[comp] )
      {
        xFree(m_puhCbf[comp]);
        m_puhCbf[comp] = NULL;
      }
      if ( m_pcTrCoeff[comp] )
      {
        xFree(m_pcTrCoeff[comp]);
        m_pcTrCoeff[comp] = NULL;
      }
      if ( m_explicitRdpcmMode[comp] )
      {
        xFree(m_explicitRdpcmMode[comp]);
        m_explicitRdpcmMode[comp] = NULL;
      }

#if ADAPTIVE_QP_SELECTION
      if (!m_ArlCoeffIsAliasedAllocation)
      {
        if ( m_pcArlCoeff[comp] )
        {
          xFree(m_pcArlCoeff[comp]);
          m_pcArlCoeff[comp] = NULL;
        }
      }
#endif

      if ( m_pcIPCMSample[comp] )
      {
        xFree(m_pcIPCMSample[comp]);
        m_pcIPCMSample[comp] = NULL;
      }
    }
    if ( m_pbIPCMFlag )
    {
      xFree(m_pbIPCMFlag );
      m_pbIPCMFlag = NULL;
    }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      if ( m_apiMVPIdx[rpl] )
      {
        delete[] m_apiMVPIdx[rpl];
        m_apiMVPIdx[rpl] = NULL;
      }
      if ( m_apiMVPNum[rpl] )
      {
        delete[] m_apiMVPNum[rpl];
        m_apiMVPNum[rpl] = NULL;
      }
    }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_acCUMvField[rpl].destroy();
    }
  }

  m_pcPic              = NULL;
  m_pcSlice            = NULL;

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;

}

Bool TComDataCU::CUIsFromSameTile            ( const TComDataCU *pCU /* Can be NULL */) const
{
  return pCU!=NULL &&
         pCU->getSlice() != NULL &&
         m_pcPic->getPicSym()->getTileIdxMap( pCU->getCtuRsAddr() ) == m_pcPic->getPicSym()->getTileIdxMap(getCtuRsAddr());
}

Bool TComDataCU::CUIsFromSameSliceAndTile    ( const TComDataCU *pCU /* Can be NULL */) const
{
  return pCU!=NULL &&
         pCU->getSlice() != NULL &&
         pCU->getSlice()->getSliceCurStartCtuTsAddr() == getSlice()->getSliceCurStartCtuTsAddr() &&
         m_pcPic->getPicSym()->getTileIdxMap( pCU->getCtuRsAddr() ) == m_pcPic->getPicSym()->getTileIdxMap(getCtuRsAddr())
         ;
}

Bool TComDataCU::CUIsFromSameSliceTileAndWavefrontRow( const TComDataCU *pCU /* Can be NULL */) const
{
  return CUIsFromSameSliceAndTile(pCU)
         && (!getSlice()->getPPS()->getEntropyCodingSyncEnabledFlag() || getPic()->getCtu(getCtuRsAddr())->getCUPelY() == getPic()->getCtu(pCU->getCtuRsAddr())->getCUPelY());
}

Bool TComDataCU::isLastSubCUOfCtu(const UInt absPartIdx) const
{
  const TComSPS &sps=*(getSlice()->getSPS());

  const UInt picWidth = sps.getPicWidthInLumaSamples();
  const UInt picHeight = sps.getPicHeightInLumaSamples();
  const UInt granularityWidth = sps.getMaxCUWidth();

