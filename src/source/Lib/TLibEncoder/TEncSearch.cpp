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

/** \file     TEncSearch.cpp
 \brief    encoder search class
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TEncSearch.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/Debug.h"
#include <math.h>
#include <limits>


//! \ingroup TLibEncoder
//! \{

static const TComMv s_acMvRefineH[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static const TComMv s_acMvRefineQ[9] =
{
  TComMv(  0,  0 ), // 0
  TComMv(  0, -1 ), // 1
  TComMv(  0,  1 ), // 2
  TComMv( -1, -1 ), // 5
  TComMv(  1, -1 ), // 6
  TComMv( -1,  0 ), // 3
  TComMv(  1,  0 ), // 4
  TComMv( -1,  1 ), // 7
  TComMv(  1,  1 )  // 8
};

static Void offsetSubTUCBFs(TComTU &rTu, const ComponentID compID)
{
        TComDataCU *pcCU              = rTu.getCU();
  const UInt        uiTrDepth         = rTu.GetTransformDepthRel();
  const UInt        uiAbsPartIdx      = rTu.GetAbsPartIdxTU(compID);
  const UInt        partIdxesPerSubTU = rTu.GetAbsPartIdxNumParts(compID) >> 1;

  //move the CBFs down a level and set the parent CBF

  UChar subTUCBF[2];
  UChar combinedSubTUCBF = 0;

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);

    subTUCBF[subTU]   = pcCU->getCbf(subTUAbsPartIdx, compID, uiTrDepth);
    combinedSubTUCBF |= subTUCBF[subTU];
  }

  for (UInt subTU = 0; subTU < 2; subTU++)
  {
    const UInt subTUAbsPartIdx = uiAbsPartIdx + (subTU * partIdxesPerSubTU);
    const UChar compositeCBF = (subTUCBF[subTU] << 1) | combinedSubTUCBF;

    pcCU->setCbfPartRange((compositeCBF << uiTrDepth), compID, subTUAbsPartIdx, partIdxesPerSubTU);
  }
}


TEncSearch::TEncSearch()
: m_puhQTTempTrIdx(NULL)
, m_pcQTTempTComYuv(NULL)
, m_pcEncCfg (NULL)
, m_pcTrQuant (NULL)
, m_pcRdCost (NULL)
, m_pcEntropyCoder (NULL)
, m_iSearchRange (0)
, m_bipredSearchRange (0)
, m_motionEstimationSearchMethod (MESEARCH_FULL)
, m_pppcRDSbacCoder (NULL)
, m_pcRDGoOnSbacCoder (NULL)
, m_pTempPel (NULL)
, m_isInitialized (false)
{
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    m_ppcQTTempCoeff[ch]                           = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]                        = NULL;
#endif
    m_puhQTTempCbf[ch]                             = NULL;
    m_phQTTempCrossComponentPredictionAlpha[ch]    = NULL;
    m_pSharedPredTransformSkip[ch]                 = NULL;
    m_pcQTTempTUCoeff[ch]                          = NULL;
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = NULL;
#endif
    m_puhQTTempTransformSkipFlag[ch]               = NULL;
  }

  for (Int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (Int));
  }
  for (Int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (UInt) );
  }

  setWpScalingDistParam( NULL, -1, REF_PIC_LIST_X );
}


Void TEncSearch::destroy()
{
  assert (m_isInitialized);
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  if ( m_pcEncCfg )
  {
    const UInt uiNumLayersAllocated = m_pcEncCfg->getQuadtreeTULog2MaxSize()-m_pcEncCfg->getQuadtreeTULog2MinSize()+1;

    for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for (UInt layer = 0; layer < uiNumLayersAllocated; layer++)
      {
        delete[] m_ppcQTTempCoeff[ch][layer];
#if ADAPTIVE_QP_SELECTION
        delete[] m_ppcQTTempArlCoeff[ch][layer];
#endif
      }
      delete[] m_ppcQTTempCoeff[ch];
      delete[] m_puhQTTempCbf[ch];
#if ADAPTIVE_QP_SELECTION
      delete[] m_ppcQTTempArlCoeff[ch];
#endif
    }

    for( UInt layer = 0; layer < uiNumLayersAllocated; layer++ )
    {
      m_pcQTTempTComYuv[layer].destroy();
    }
  }

  delete[] m_puhQTTempTrIdx;
  delete[] m_pcQTTempTComYuv;

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    delete[] m_pSharedPredTransformSkip[ch];
    delete[] m_pcQTTempTUCoeff[ch];
#if ADAPTIVE_QP_SELECTION
    delete[] m_ppcQTTempTUArlCoeff[ch];
#endif
    delete[] m_phQTTempCrossComponentPredictionAlpha[ch];
    delete[] m_puhQTTempTransformSkipFlag[ch];
  }
  m_pcQTTempTransformSkipTComYuv.destroy();

  m_tmpYuvPred.destroy();
  m_isInitialized = false;
}

TEncSearch::~TEncSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}




Void TEncSearch::init(TEncCfg*       pcEncCfg,
                      TComTrQuant*   pcTrQuant,
                      Int            iSearchRange,
                      Int            bipredSearchRange,
                      MESearchMethod motionEstimationSearchMethod,
                      const UInt     maxCUWidth,
                      const UInt     maxCUHeight,
                      const UInt     maxTotalCUDepth,
                      TEncEntropy*   pcEntropyCoder,
                      TComRdCost*    pcRdCost,
                      TEncSbac***    pppcRDSbacCoder,
                      TEncSbac*      pcRDGoOnSbacCoder
                      )
{
  assert (!m_isInitialized);
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_iSearchRange                 = iSearchRange;
  m_bipredSearchRange            = bipredSearchRange;
  m_motionEstimationSearchMethod = motionEstimationSearchMethod;
  m_pcEntropyCoder               = pcEntropyCoder;
  m_pcRdCost                     = pcRdCost;

  m_pppcRDSbacCoder              = pppcRDSbacCoder;
  m_pcRDGoOnSbacCoder            = pcRDGoOnSbacCoder;

  for (UInt iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++)
  {
    for (UInt iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++)
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  // initialize motion cost
  for( Int iNum = 0; iNum < AMVP_MAX_NUM_CANDS+1; iNum++)
  {
    for( Int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++)
    {
      if (iIdx < iNum)
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits(iIdx, iNum);
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_INT;
      }
    }
  }

  const ChromaFormat cform=pcEncCfg->getChromaFormatIdc();
  initTempBuff(cform);

  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];

  const UInt uiNumLayersToAllocate = pcEncCfg->getQuadtreeTULog2MaxSize()-pcEncCfg->getQuadtreeTULog2MinSize()+1;
  const UInt uiNumPartitions = 1<<(maxTotalCUDepth<<1);
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt csx=::getComponentScaleX(ComponentID(ch), cform);
    const UInt csy=::getComponentScaleY(ComponentID(ch), cform);
    m_ppcQTTempCoeff[ch] = new TCoeff* [uiNumLayersToAllocate];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempArlCoeff[ch]  = new TCoeff*[uiNumLayersToAllocate];
#endif
    m_puhQTTempCbf[ch] = new UChar  [uiNumPartitions];

    for (UInt layer = 0; layer < uiNumLayersToAllocate; layer++)
    {
      m_ppcQTTempCoeff[ch][layer] = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy)];
#if ADAPTIVE_QP_SELECTION
      m_ppcQTTempArlCoeff[ch][layer]  = new TCoeff[(maxCUWidth*maxCUHeight)>>(csx+csy) ];
#endif
    }

    m_phQTTempCrossComponentPredictionAlpha[ch]    = new SChar  [uiNumPartitions];
    m_pSharedPredTransformSkip[ch]                 = new Pel   [MAX_CU_SIZE*MAX_CU_SIZE];
    m_pcQTTempTUCoeff[ch]                          = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#if ADAPTIVE_QP_SELECTION
    m_ppcQTTempTUArlCoeff[ch]                      = new TCoeff[MAX_CU_SIZE*MAX_CU_SIZE];
#endif
    m_puhQTTempTransformSkipFlag[ch]               = new UChar [uiNumPartitions];
  }
  m_puhQTTempTrIdx   = new UChar  [uiNumPartitions];
  m_pcQTTempTComYuv  = new TComYuv[uiNumLayersToAllocate];
  for( UInt ui = 0; ui < uiNumLayersToAllocate; ++ui )
  {
    m_pcQTTempTComYuv[ui].create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
  }
  m_pcQTTempTransformSkipTComYuv.create( maxCUWidth, maxCUHeight, pcEncCfg->getChromaFormatIdc() );
  m_tmpYuvPred.create(MAX_CU_SIZE, MAX_CU_SIZE, pcEncCfg->getChromaFormatIdc());
  m_isInitialized = true;
}


__inline Void TEncSearch::xTZSearchHelp( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance )
{
  Distortion  uiSad = 0;

  const Pel* const  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iYStride + iSearchX;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( pcPatternKey, piRefSrch, rcStruct.iYStride,  m_cDistParam );

  setDistParamComp(COMPONENT_Y);

  // distortion
  m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
  m_cDistParam.m_maximumDistortionForEarlyExit = rcStruct.uiBestSad;

  if((m_pcEncCfg->getRestrictMESampling() == false) && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE)
  {
    Int isubShift = 0;
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      if ( m_cDistParam.iRows > 32 )
      {
        m_cDistParam.iSubShift = 4;
      }
      else if ( m_cDistParam.iRows > 16 )
      {
        m_cDistParam.iSubShift = 3;
      }
      else if ( m_cDistParam.iRows > 8 )
      {
        m_cDistParam.iSubShift = 2;
      }
      else
      {
        m_cDistParam.iSubShift = 1;
      }

      Distortion uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
      if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        uiSad += uiTempSad >>  m_cDistParam.iSubShift;
        while(m_cDistParam.iSubShift > 0)
        {
          isubShift         = m_cDistParam.iSubShift -1;
          m_cDistParam.pOrg = pcPatternKey->getROIY() + (pcPatternKey->getPatternLStride() << isubShift);
          m_cDistParam.pCur = piRefSrch + (rcStruct.iYStride << isubShift);
          uiTempSad = m_cDistParam.DistFunc( &m_cDistParam );
          uiSad += uiTempSad >>  m_cDistParam.iSubShift;
          if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.iSubShift--;
        }

        if(m_cDistParam.iSubShift == 0)
        {
          uiSad += uiBitCost;
          if( uiSad < rcStruct.uiBestSad )
          {
            rcStruct.uiBestSad      = uiSad;
            rcStruct.iBestX         = iSearchX;
            rcStruct.iBestY         = iSearchY;
            rcStruct.uiBestDistance = uiDistance;
            rcStruct.uiBestRound    = 0;
            rcStruct.ucPointNr      = ucPointNr;
            m_cDistParam.m_maximumDistortionForEarlyExit = uiSad;
          }
        }
      }
    }
  }
  else
  {
    // fast encoder decision: use subsampled SAD when rows > 8 for integer ME
    if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE3 )
    {
      if ( m_cDistParam.iRows > 8 )
      {
        m_cDistParam.iSubShift = 1;
      }
    }

    uiSad = m_cDistParam.DistFunc( &m_cDistParam );

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if( uiSad < rcStruct.uiBestSad )
    {
      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY );

      if( uiSad < rcStruct.uiBestSad )
      {
        rcStruct.uiBestSad      = uiSad;
        rcStruct.iBestX         = iSearchX;
        rcStruct.iBestY         = iSearchY;
        rcStruct.uiBestDistance = uiDistance;
        rcStruct.uiBestRound    = 0;
        rcStruct.ucPointNr      = ucPointNr;
        m_cDistParam.m_maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}

__inline Void TEncSearch::xTZ2PointSearch( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const TComMv* const pcMvSrchRngLT, const TComMv* const pcMvSrchRngRB )
{
  Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  Int iStartX = rcStruct.iBestX;
  Int iStartY = rcStruct.iBestY;
  switch( rcStruct.ucPointNr )
  {
    case 1:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY, 0, 2 );
      }
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
    }
      break;
    case 2:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 3:
    {
      if ( (iStartY - 1) >= iSrchRngVerTop )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY - 1, 0, 2 );
      }
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
    }
      break;
    case 4:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY - 1, 0, 2 );
        }
      }
    }
      break;
    case 5:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        if ( (iStartY - 1) >= iSrchRngVerTop )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY - 1, 0, 2 );
        }
        if ( (iStartY + 1) <= iSrchRngVerBottom )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 6:
    {
      if ( (iStartX - 1) >= iSrchRngHorLeft )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY , 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    case 7:
    {
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        if ( (iStartX - 1) >= iSrchRngHorLeft )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX - 1, iStartY + 1, 0, 2 );
        }
        if ( (iStartX + 1) <= iSrchRngHorRight )
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY + 1, 0, 2 );
        }
      }
    }
      break;
    case 8:
    {
      if ( (iStartX + 1) <= iSrchRngHorRight )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX + 1, iStartY, 0, 2 );
      }
      if ( (iStartY + 1) <= iSrchRngVerBottom )
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iStartY + 1, 0, 2 );
      }
    }
      break;
    default:
    {
      assert( false );
    }
      break;
  } // switch( rcStruct.ucPointNr )
}




__inline Void TEncSearch::xTZ8PointSquareSearch( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const TComMv* const pcMvSrchRngLT, const TComMv* const pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist )
{
  const Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  const Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  const Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  const Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= iSrchRngVerTop ) // check top
  {
    if ( iLeft >= iSrchRngHorLeft ) // check top left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= iSrchRngHorRight ) // check top right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= iSrchRngHorLeft ) // check middle left
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= iSrchRngHorRight ) // check middle right
  {
    xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= iSrchRngVerBottom ) // check bottom
  {
    if ( iLeft >= iSrchRngHorLeft ) // check bottom left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= iSrchRngHorRight ) // check bottom right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}




__inline Void TEncSearch::xTZ8PointDiamondSearch( const TComPattern*const  pcPatternKey,
                                                  IntTZSearchStruct& rcStruct,
                                                  const TComMv*const  pcMvSrchRngLT,
                                                  const TComMv*const  pcMvSrchRngRB,
                                                  const Int iStartX,
                                                  const Int iStartY,
                                                  const Int iDist,
                                                  const Bool bCheckCornersAtDist1 )
{
  const Int   iSrchRngHorLeft   = pcMvSrchRngLT->getHor();
  const Int   iSrchRngHorRight  = pcMvSrchRngRB->getHor();
  const Int   iSrchRngVerTop    = pcMvSrchRngLT->getVer();
  const Int   iSrchRngVerBottom = pcMvSrchRngRB->getVer();

  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  assert ( iDist != 0 );
  const Int iTop        = iStartY - iDist;
  const Int iBottom     = iStartY + iDist;
  const Int iLeft       = iStartX - iDist;
  const Int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 )
  {
    if ( iTop >= iSrchRngVerTop ) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= iSrchRngHorLeft) // check top-left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iTop, 1, iDist );
        }
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        if ( iRight <= iSrchRngHorRight ) // check middle right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iTop, 3, iDist );
        }
      }
      else
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
      }
    }
    if ( iLeft >= iSrchRngHorLeft ) // check middle left
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= iSrchRngHorRight ) // check middle right
    {
      xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= iSrchRngVerBottom ) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= iSrchRngHorLeft) // check top-left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iBottom, 6, iDist );
        }
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        if ( iRight <= iSrchRngHorRight ) // check middle right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iBottom, 8, iDist );
        }
      }
      else
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
      }
    }
  }
  else
  {
    if ( iDist <= 8 )
    {
      const Int iTop_2      = iStartY - (iDist>>1);
      const Int iBottom_2   = iStartY + (iDist>>1);
      const Int iLeft_2     = iStartX - (iDist>>1);
      const Int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= iSrchRngVerTop ) // check half top
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= iSrchRngVerBottom ) // check half bottom
        {
          if ( iLeft_2 >= iSrchRngHorLeft ) // check half left
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= iSrchRngHorRight ) // check half right
          {
            xTZSearchHelp( pcPatternKey, rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= iSrchRngVerTop && iLeft >= iSrchRngHorLeft &&
          iRight <= iSrchRngHorRight && iBottom <= iSrchRngVerBottom ) // check border
      {
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        for ( Int index = 1; index < 4; index++ )
        {
          const Int iPosYT = iTop    + ((iDist>>2) * index);
          const Int iPosYB = iBottom - ((iDist>>2) * index);
          const Int iPosXL = iStartX - ((iDist>>2) * index);
          const Int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= iSrchRngVerTop ) // check top
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= iSrchRngHorLeft ) // check left
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= iSrchRngHorRight ) // check right
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= iSrchRngVerBottom ) // check bottom
        {
          xTZSearchHelp( pcPatternKey, rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( Int index = 1; index < 4; index++ )
        {
          const Int iPosYT = iTop    + ((iDist>>2) * index);
          const Int iPosYB = iBottom - ((iDist>>2) * index);
          const Int iPosXL = iStartX - ((iDist>>2) * index);
          const Int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= iSrchRngVerTop ) // check top
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= iSrchRngVerBottom ) // check bottom
          {
            if ( iPosXL >= iSrchRngHorLeft ) // check left
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= iSrchRngHorRight ) // check right
            {
              xTZSearchHelp( pcPatternKey, rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion TEncSearch::xPatternRefinement( TComPattern* pcPatternKey,
                                           TComMv baseRefMv,
                                           Int iFrac, TComMv& rcMvFrac,
                                           Bool bAllowUseOfHadamard
                                         )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  UInt        uiDirecBest = 0;

  Pel*  piRefPos;
  Int iRefStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);

  m_pcRdCost->setDistParam( pcPatternKey, m_filteredBlock[0][0].getAddr(COMPONENT_Y), iRefStride, 1, m_cDistParam, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const TComMv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);

#if MCTS_ENC_CHECK
  UInt maxRefinements = 9;
  Int mvShift = 2;

  // filter length of sub-sample generation filter to be considered
  const UInt LumaLTSampleOffset = 3;
  const UInt LumaRBSampleOffset = 4;

  if (m_pcEncCfg->getTMCTSSEITileConstraint())
  {
    // if close to tile borders
    if ( pcPatternKey->getROIYPosX() + (baseRefMv.getHor() >> mvShift ) < pcPatternKey->getTileLeftTopPelPosX() +  LumaLTSampleOffset ||
         pcPatternKey->getROIYPosY() + (baseRefMv.getVer() >> mvShift ) < pcPatternKey->getTileLeftTopPelPosY() +  LumaLTSampleOffset ||
         pcPatternKey->getROIYPosX() + (baseRefMv.getHor() >> mvShift)  > pcPatternKey->getTileRightBottomPelPosX() - pcPatternKey->getROIYWidth() - LumaRBSampleOffset ||
         pcPatternKey->getROIYPosY() + (baseRefMv.getVer() >> mvShift)  > pcPatternKey->getTileRightBottomPelPosY() - pcPatternKey->getROIYHeight() - LumaRBSampleOffset  
       )
    {
      // only allow full pel positions to avoid filter dependency
      maxRefinements = 1;
    }
  }  

  for (UInt i = 0; i < maxRefinements; i++)
#else
  for (UInt i = 0; i < 9; i++)
#endif
  {
    TComMv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    Int horVal = cMvTest.getHor() * iFrac;
    Int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[ verVal & 3 ][ horVal & 3 ].getAddr(COMPONENT_Y);
    if ( horVal == 2 && ( verVal & 1 ) == 0 )
    {
      piRefPos += 1;
    }
    if ( ( horVal & 1 ) == 0 && verVal == 2 )
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;

    setDistParamComp(COMPONENT_Y);

    m_cDistParam.pCur = piRefPos;
    m_cDistParam.bitDepth = pcPatternKey->getBitDepthY();
    uiDist = m_cDistParam.DistFunc( &m_cDistParam );
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.getHor(), cMvTest.getVer() );

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
      m_cDistParam.m_maximumDistortionForEarlyExit = uiDist;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}



Void
TEncSearch::xEncSubdivCbfQT(TComTU      &rTu,
                            Bool         bLuma,
                            Bool         bChroma )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx         = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth            = rTu.GetTransformDepthRel();
  const UInt uiTrMode             = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt uiSubdiv             = ( uiTrMode > uiTrDepth ? 1 : 0 );
  const UInt uiLog2LumaTrafoSize  = rTu.GetLog2LumaTrSize();

  if( pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_NxN && uiTrDepth == 0 )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize > pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() )
  {
    assert( uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() )
  {
    assert( !uiSubdiv );
  }
  else if( uiLog2LumaTrafoSize == pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) )
  {
    assert( !uiSubdiv );
  }
  else
  {
    assert( uiLog2LumaTrafoSize > pcCU->getQuadtreeTULog2MinSizeInCU(uiAbsPartIdx) );
    if( bLuma )
    {
      m_pcEntropyCoder->encodeTransformSubdivFlag( uiSubdiv, 5 - uiLog2LumaTrafoSize );
    }
  }

  if ( bChroma )
  {
    const UInt numberValidComponents = getNumberValidComponents(rTu.GetChromaFormat());
    for (UInt ch=COMPONENT_Cb; ch<numberValidComponents; ch++)
    {
      const ComponentID compID=ComponentID(ch);
      if( rTu.ProcessingAllQuadrants(compID) && (uiTrDepth==0 || pcCU->getCbf( uiAbsPartIdx, compID, uiTrDepth-1 ) ))
      {
        m_pcEntropyCoder->encodeQtCbf(rTu, compID, (uiSubdiv == 0));
      }
    }
  }

  if( uiSubdiv )
  {
    TComTURecurse tuRecurse(rTu, false);
    do
    {
      xEncSubdivCbfQT( tuRecurse, bLuma, bChroma );
    } while (tuRecurse.nextSection(rTu));
  }
  else
  {
    //===== Cbfs =====
    if( bLuma )
    {
      m_pcEntropyCoder->encodeQtCbf( rTu, COMPONENT_Y, true );
    }
  }
}




Void
TEncSearch::xEncCoeffQT(TComTU &rTu,
                        const ComponentID  component,
                        Bool         bRealCoeff )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();

  const UInt  uiTrMode        = pcCU->getTransformIdx( uiAbsPartIdx );
  const UInt  uiSubdiv        = ( uiTrMode > uiTrDepth ? 1 : 0 );

  if( uiSubdiv )
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xEncCoeffQT( tuRecurseChild, component, bRealCoeff );
    } while (tuRecurseChild.nextSection(rTu) );
  }
  else if (rTu.ProcessComponentSection(component))
  {
    //===== coefficients =====
    const UInt  uiLog2TrafoSize = rTu.GetLog2LumaTrSize();
    UInt    uiCoeffOffset   = rTu.getCoefficientOffset(component);
    UInt    uiQTLayer       = pcCU->getSlice()->getSPS()->getQuadtreeTULog2MaxSize() - uiLog2TrafoSize;
    TCoeff* pcCoeff         = bRealCoeff ? pcCU->getCoeff(component) : m_ppcQTTempCoeff[component][uiQTLayer];

    if (isChroma(component) && (pcCU->getCbf( rTu.GetAbsPartIdxTU(), COMPONENT_Y, uiTrMode ) != 0) && pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
    {
      m_pcEntropyCoder->encodeCrossComponentPrediction( rTu, component );
    }

    m_pcEntropyCoder->encodeCoeffNxN( rTu, pcCoeff+uiCoeffOffset, component );
  }
}




Void
TEncSearch::xEncIntraHeader( TComDataCU*  pcCU,
                            UInt         uiTrDepth,
                            UInt         uiAbsPartIdx,
                            Bool         bLuma,
                            Bool         bChroma )
{
  if( bLuma )
  {
    // CU header
    if( uiAbsPartIdx == 0 )
    {
      if( !pcCU->getSlice()->isIntra() )
      {
        if (pcCU->getSlice()->getPPS()->getTransquantBypassEnabledFlag())
        {
          m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, 0, true );
        }
        m_pcEntropyCoder->encodeSkipFlag( pcCU, 0, true );
        m_pcEntropyCoder->encodePredMode( pcCU, 0, true );
      }
      m_pcEntropyCoder  ->encodePartSize( pcCU, 0, pcCU->getDepth(0), true );

      if (pcCU->isIntra(0) && pcCU->getPartitionSize(0) == SIZE_2Nx2N )
      {
        m_pcEntropyCoder->encodeIPCMInfo( pcCU, 0, true );

        if ( pcCU->getIPCMFlag (0))
        {
          return;
        }
      }
    }
    // luma prediction mode
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N )
    {
      if (uiAbsPartIdx==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, 0 );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      if (uiTrDepth>0 && (uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeLuma ( pcCU, uiAbsPartIdx );
      }
    }
  }

  if( bChroma )
  {
    if( pcCU->getPartitionSize(0) == SIZE_2Nx2N || !enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()))
    {
      if(uiAbsPartIdx==0)
      {
         m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
    else
    {
      UInt uiQNumParts = pcCU->getTotalNumPart() >> 2;
      assert(uiTrDepth>0);
      if ((uiAbsPartIdx%uiQNumParts)==0)
      {
        m_pcEntropyCoder->encodeIntraDirModeChroma ( pcCU, uiAbsPartIdx );
      }
    }
  }
}




UInt
TEncSearch::xGetIntraBitsQT(TComTU &rTu,
                            Bool         bLuma,
                            Bool         bChroma,
                            Bool         bRealCoeff /* just for test */ )
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const UInt uiTrDepth=rTu.GetTransformDepthRel();
  m_pcEntropyCoder->resetBits();
  xEncIntraHeader ( pcCU, uiTrDepth, uiAbsPartIdx, bLuma, bChroma );
  xEncSubdivCbfQT ( rTu, bLuma, bChroma );

  if( bLuma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Y,      bRealCoeff );
  }
  if( bChroma )
  {
    xEncCoeffQT   ( rTu, COMPONENT_Cb,  bRealCoeff );
    xEncCoeffQT   ( rTu, COMPONENT_Cr,  bRealCoeff );
  }
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();

  return uiBits;
}

UInt TEncSearch::xGetIntraBitsQTChroma(TComTU &rTu,
                                       ComponentID compID,
                                       Bool         bRealCoeff /* just for test */ )
{
  m_pcEntropyCoder->resetBits();
  xEncCoeffQT   ( rTu, compID,  bRealCoeff );
  UInt   uiBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  return uiBits;
}

Void TEncSearch::xIntraCodingTUBlock(       TComYuv*    pcOrgYuv,
                                            TComYuv*    pcPredYuv,
                                            TComYuv*    pcResiYuv,
                                            Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                      const Bool        checkCrossCPrediction,
                                            Distortion& ruiDist,
                                      const ComponentID compID,
                                            TComTU&     rTu
                                      DEBUG_STRING_FN_DECLARE(sDebug)
                                           ,Int         default0Save1Load2
                                     )
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }
  const Bool           bIsLuma          = isLuma(compID);
  const TComRectangle &rect             = rTu.getRect(compID);
        TComDataCU    *pcCU             = rTu.getCU();
  const UInt           uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const TComSPS       &sps              = *(pcCU->getSlice()->getSPS());

  const UInt           uiTrDepth        = rTu.GetTransformDepthRelAdj(compID);
  const UInt           uiFullDepth      = rTu.GetTransformDepthTotal();
  const UInt           uiLog2TrSize     = rTu.GetLog2LumaTrSize();
  const ChromaFormat   chFmt            = pcOrgYuv->getChromaFormat();
  const ChannelType    chType           = toChannelType(compID);
  const Int            bitDepth         = sps.getBitDepth(chType);

  const UInt           uiWidth          = rect.width;
  const UInt           uiHeight         = rect.height;
  const UInt           uiStride         = pcOrgYuv ->getStride (compID);
        Pel           *piOrg            = pcOrgYuv ->getAddr( compID, uiAbsPartIdx );
        Pel           *piPred           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piResi           = pcResiYuv->getAddr( compID, uiAbsPartIdx );
        Pel           *piReco           = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const UInt           uiQTLayer        = sps.getQuadtreeTULog2MaxSize() - uiLog2TrSize;
        Pel           *piRecQt          = m_pcQTTempTComYuv[ uiQTLayer ].getAddr( compID, uiAbsPartIdx );
  const UInt           uiRecQtStride    = m_pcQTTempTComYuv[ uiQTLayer ].getStride(compID);
  const UInt           uiZOrder         = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
        Pel           *piRecIPred       = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), uiZOrder );
        UInt           uiRecIPredStride = pcCU->getPic()->getPicYuvRec()->getStride  ( compID );
        TCoeff        *pcCoeff          = m_ppcQTTempCoeff[compID][uiQTLayer] + rTu.getCoefficientOffset(compID);
        Bool           useTransformSkip = pcCU->getTransformSkip(uiAbsPartIdx, compID);

#if ADAPTIVE_QP_SELECTION
        TCoeff        *pcArlCoeff       = m_ppcQTTempArlCoeff[compID][ uiQTLayer ] + rTu.getCoefficientOffset(compID);
#endif

  const UInt           uiChPredMode     = pcCU->getIntraDir( chType, uiAbsPartIdx );
  const UInt           partsPerMinCU    = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
  const UInt           uiChCodedMode    = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
  const UInt           uiChFinalMode    = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  const Int            blkX                                 = g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            blkY                                 = g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsPartIdx ] ];
  const Int            bufferOffset                         = blkX + (blkY * MAX_CU_SIZE);
        Pel  *const    encoderLumaResidual                  = resiLuma[RESIDUAL_ENCODER_SIDE ] + bufferOffset;
        Pel  *const    reconstructedLumaResidual            = resiLuma[RESIDUAL_RECONSTRUCTED] + bufferOffset;
  const Bool           bUseCrossCPrediction                 = isChroma(compID) && (uiChPredMode == DM_CHROMA_IDX) && checkCrossCPrediction;
  const Bool           bUseReconstructedResidualForEstimate = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();
        Pel *const     lumaResidualForEstimate              = bUseReconstructedResidualForEstimate ? reconstructedLumaResidual : encoderLumaResidual;

#if DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
#endif

  //===== init availability pattern =====
  DEBUG_STRING_NEW(sTemp)

#if !DEBUG_STRING
  if( default0Save1Load2 != 2 )
#endif
  {
    const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag());

    initIntraPatternChType( rTu, compID, bUseFilteredPredictions DEBUG_STRING_PASS_INTO(sDebug) );

    //===== get prediction signal =====
    predIntraAng( compID, uiChFinalMode, piOrg, uiStride, piPred, uiStride, rTu, bUseFilteredPredictions );

    // save prediction
    if( default0Save1Load2 == 1 )
    {
      Pel*  pPred   = piPred;
      Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
      Int k = 0;
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        for( UInt uiX = 0; uiX < uiWidth; uiX++ )
        {
          pPredBuf[ k ++ ] = pPred[ uiX ];
        }
        pPred += uiStride;
      }
    }
  }
#if !DEBUG_STRING
  else
  {
    // load prediction
    Pel*  pPred   = piPred;
    Pel*  pPredBuf = m_pSharedPredTransformSkip[compID];
    Int k = 0;
    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pPred[ uiX ] = pPredBuf[ k ++ ];
      }
      pPred += uiStride;
    }
  }
#endif

  //===== get residual signal =====
  {
    // get residual
    Pel*  pOrg    = piOrg;
    Pel*  pPred   = piPred;
    Pel*  pResi   = piResi;

    for( UInt uiY = 0; uiY < uiHeight; uiY++ )
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        pResi[ uiX ] = pOrg[ uiX ] - pPred[ uiX ];
      }

      pOrg  += uiStride;
      pResi += uiStride;
      pPred += uiStride;
    }
  }

  if (pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag())
  {
    if (bUseCrossCPrediction)
    {
      if (xCalcCrossComponentPredictionAlpha( rTu, compID, lumaResidualForEstimate, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride ) == 0)
      {
        return;
      }
      TComTrQuant::crossComponentPrediction ( rTu, compID, reconstructedLumaResidual, piResi, piResi, uiWidth, uiHeight, MAX_CU_SIZE, uiStride, uiStride, false );
    }
    else if (isLuma(compID) && !bUseReconstructedResidualForEstimate)
    {
      xStoreCrossComponentPredictionResult( encoderLumaResidual, piResi, rTu, 0, 0, MAX_CU_SIZE, uiStride );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  if( useTransformSkip ? m_pcEncCfg->getUseRDOQTS() : m_pcEncCfg->getUseRDOQ() )
  {
    COEFF_SCAN_TYPE scanType = COEFF_SCAN_TYPE(pcCU->getCoefScanIdx(uiAbsPartIdx, uiWidth, uiHeight, compID));
    m_pcEntropyCoder->estimateBit( m_pcTrQuant->m_pcEstBitsSbac, uiWidth, uiHeight, chType, scanType );
  }

  //--- transform and quantization ---
  TCoeff uiAbsSum = 0;
  if (bIsLuma)
  {
    pcCU       ->setTrIdxSubParts ( uiTrDepth, uiAbsPartIdx, uiFullDepth );
  }

  const QpParam cQP(*pcCU, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda     (compID);
#endif

