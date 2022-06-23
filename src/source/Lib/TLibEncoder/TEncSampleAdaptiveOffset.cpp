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

/**
 \file     TEncSampleAdaptiveOffset.cpp
 \brief       estimation part of sample adaptive offset class
 */
#include "TEncSampleAdaptiveOffset.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup TLibEncoder
//! \{


//! rounding with IBDI
inline Double xRoundIbdi2(Int bitDepth, Double x)
{
  return ((x)>0) ? (Int)(((Int)(x)+(1<<(bitDepth-8-1)))/(1<<(bitDepth-8))) : ((Int)(((Int)(x)-(1<<(bitDepth-8-1)))/(1<<(bitDepth-8))));
}

inline Double xRoundIbdi(Int bitDepth, Double x)
{
  return (bitDepth > 8 ? xRoundIbdi2(bitDepth, (x)) : ((x)>=0 ? ((Int)((x)+0.5)) : ((Int)((x)-0.5)))) ;
}


TEncSampleAdaptiveOffset::TEncSampleAdaptiveOffset()
{
  m_pppcRDSbacCoder = NULL;
  m_pcRDGoOnSbacCoder = NULL;
  m_pppcBinCoderCABAC = NULL;
  m_statData = NULL;
  m_preDBFstatData = NULL;
}

TEncSampleAdaptiveOffset::~TEncSampleAdaptiveOffset()
{
  destroyEncData();
}

Void TEncSampleAdaptiveOffset::createEncData(Bool isPreDBFSamplesUsed)
{

  //cabac coder for RDO
  m_pppcRDSbacCoder = new TEncSbac* [NUM_SAO_CABACSTATE_LABELS];
#if FAST_BIT_EST
  m_pppcBinCoderCABAC = new TEncBinCABACCounter* [NUM_SAO_CABACSTATE_LABELS];
#else
  m_pppcBinCoderCABAC = new TEncBinCABAC* [NUM_SAO_CABACSTATE_LABELS];
#endif

  for(Int cs=0; cs < NUM_SAO_CABACSTATE_LABELS; cs++)
  {
    m_pppcRDSbacCoder[cs] = new TEncSbac;
#if FAST_BIT_EST
    m_pppcBinCoderCABAC[cs] = new TEncBinCABACCounter;
#else
    m_pppcBinCoderCABAC[cs] = new TEncBinCABAC;
#endif
    m_pppcRDSbacCoder   [cs]->init( m_pppcBinCoderCABAC [cs] );
  }


  //statistics
  m_statData = new SAOStatData**[m_numCTUsPic];
  for(Int i=0; i< m_numCTUsPic; i++)
  {
    m_statData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
    for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      m_statData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
    }
  }
  if(isPreDBFSamplesUsed)
  {
    m_preDBFstatData = new SAOStatData**[m_numCTUsPic];
    for(Int i=0; i< m_numCTUsPic; i++)
    {
      m_preDBFstatData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
      for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        m_preDBFstatData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
      }
    }

  }

  ::memset(m_saoDisabledRate, 0, sizeof(m_saoDisabledRate));

  for(Int typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
    m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

    m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
    m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;

    if(isPreDBFSamplesUsed)
    {
      switch(typeIdc)
      {
      case SAO_TYPE_EO_0:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      case SAO_TYPE_EO_90:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_EO_135:
      case SAO_TYPE_EO_45:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_BO:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      default:
        {
          printf("Not a supported type");
          assert(0);
          exit(-1);
        }
      }
    }
  }

}

Void TEncSampleAdaptiveOffset::destroyEncData()
{
  if(m_pppcRDSbacCoder != NULL)
  {
    for (Int cs = 0; cs < NUM_SAO_CABACSTATE_LABELS; cs ++ )
    {
      delete m_pppcRDSbacCoder[cs];
    }
    delete[] m_pppcRDSbacCoder; m_pppcRDSbacCoder = NULL;
  }

  if(m_pppcBinCoderCABAC != NULL)
  {
    for (Int cs = 0; cs < NUM_SAO_CABACSTATE_LABELS; cs ++ )
    {
      delete m_pppcBinCoderCABAC[cs];
    }
    delete[] m_pppcBinCoderCABAC; m_pppcBinCoderCABAC = NULL;
  }

  if(m_statData != NULL)
  {
    for(Int i=0; i< m_numCTUsPic; i++)
    {
      for(Int compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
      {
        delete[] m_statData[i][compIdx];
      }
      delete[] m_statData[i];
    }
    delete[] m_statData; m_statData = NULL;
  }
  if(m_preDBFstatData != NULL)
  {
    for(Int i=0; i< m_numCTUsPic; i++)
    {
      for(Int compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
      {
        delete[] m_preDBFstatData[i][compIdx];
      }
      delete[] m_preDBFstatData[i];
    }
    delete[] m_preDBFstatData; m_preDBFstatData = NULL;
  }
}

Void TEncSampleAdaptiveOffset::initRDOCabacCoder(TEncSbac* pcRDGoOnSbacCoder, TComSlice* pcSlice)
{
  m_pcRDGoOnSbacCoder = pcRDGoOnSbacCoder;
  m_pcRDGoOnSbacCoder->resetEntropy(pcSlice);
  m_pcRDGoOnSbacCoder->resetBits();

  m_pcRDGoOnSbacCoder->store( m_pppcRDSbacCoder[SAO_CABACSTATE_PIC_INIT]);
}


Void TEncSampleAdaptiveOffset::SAOProcess(TComPic* pPic, Bool* sliceEnabled, const Double *lambdas, const Bool bTestSAODisableAtPictureLevel, const Double saoEncodingRate, const Double saoEncodingRateChroma, const Bool isPreDBFSamplesUsed )
{
  TComPicYuv* orgYuv= pPic->getPicYuvOrg();
  TComPicYuv* resYuv= pPic->getPicYuvRec();
  memcpy(m_lambda, lambdas, sizeof(m_lambda));
  TComPicYuv* srcYuv = m_tempPicYuv;
  resYuv->copyToPic(srcYuv);
  srcYuv->setBorderExtension(false);
  srcYuv->extendPicBorder();

  //collect statistics
  getStatistics(m_statData, orgYuv, srcYuv, pPic);
  if(isPreDBFSamplesUsed)
  {
    addPreDBFStatistics(m_statData);
  }

  //slice on/off
  decidePicParams(sliceEnabled, pPic, saoEncodingRate, saoEncodingRateChroma);
  //block on/off
  SAOBlkParam* reconParams = new SAOBlkParam[m_numCTUsPic]; //temporary parameter buffer for storing reconstructed SAO parameters
  decideBlkParams(pPic, sliceEnabled, m_statData, srcYuv, resYuv, reconParams, pPic->getPicSym()->getSAOBlkParam(), bTestSAODisableAtPictureLevel, saoEncodingRate, saoEncodingRateChroma);
  delete[] reconParams;
}

Void TEncSampleAdaptiveOffset::getPreDBFStatistics(TComPic* pPic)
{
  getStatistics(m_preDBFstatData, pPic->getPicYuvOrg(), pPic->getPicYuvRec(), pPic, true);
}

Void TEncSampleAdaptiveOffset::addPreDBFStatistics(SAOStatData*** blkStats)
{
  for(Int n=0; n< m_numCTUsPic; n++)
  {
    for(Int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      for(Int typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
      {
        blkStats[n][compIdx][typeIdc] += m_preDBFstatData[n][compIdx][typeIdc];
      }
    }
  }
}

Void TEncSampleAdaptiveOffset::getStatistics(SAOStatData*** blkStats, TComPicYuv* orgYuv, TComPicYuv* srcYuv, TComPic* pPic, Bool isCalculatePreDeblockSamples)
{
  Bool isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int ctuRsAddr= 0; ctuRsAddr < m_numCTUsPic; ctuRsAddr++)
  {
    Int yPos   = (ctuRsAddr / m_numCTUInWidth)*m_maxCUHeight;
    Int xPos   = (ctuRsAddr % m_numCTUInWidth)*m_maxCUWidth;
    Int height = (yPos + m_maxCUHeight > m_picHeight)?(m_picHeight- yPos):m_maxCUHeight;
    Int width  = (xPos + m_maxCUWidth  > m_picWidth )?(m_picWidth - xPos):m_maxCUWidth;

    pPic->getPicSym()->deriveLoopFilterBoundaryAvailibility(ctuRsAddr, isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

    //NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
    //For simplicity, here only picture boundaries are considered.

    isRightAvail      = (xPos + m_maxCUWidth  < m_picWidth );
    isBelowAvail      = (yPos + m_maxCUHeight < m_picHeight);
    isBelowRightAvail = (isRightAvail && isBelowAvail);
    isBelowLeftAvail  = ((xPos > 0) && (isBelowAvail));
    isAboveRightAvail = ((yPos > 0) && (isRightAvail));

    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      const ComponentID component = ComponentID(compIdx);

      const UInt componentScaleX = getComponentScaleX(component, pPic->getChromaFormat());
      const UInt componentScaleY = getComponentScaleY(component, pPic->getChromaFormat());

      Int  srcStride  = srcYuv->getStride(component);
      Pel* srcBlk     = srcYuv->getAddr(component) + ((yPos >> componentScaleY) * srcStride) + (xPos >> componentScaleX);

      Int  orgStride  = orgYuv->getStride(component);
      Pel* orgBlk     = orgYuv->getAddr(component) + ((yPos >> componentScaleY) * orgStride) + (xPos >> componentScaleX);

      getBlkStats(component, pPic->getPicSym()->getSPS().getBitDepth(toChannelType(component)), blkStats[ctuRsAddr][component]
                , srcBlk, orgBlk, srcStride, orgStride, (width  >> componentScaleX), (height >> componentScaleY)
                , isLeftAvail,  isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
                , isCalculatePreDeblockSamples
                );

    }
  }
}

Void TEncSampleAdaptiveOffset::resetEncoderDecisions()
{
  for (Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    for (Int tempLayer = 1; tempLayer < MAX_TLAYER; tempLayer++)
    {
      m_saoDisabledRate[compIdx][tempLayer] = 0.0;
    }
  }
}

Void TEncSampleAdaptiveOffset::decidePicParams(Bool* sliceEnabled, const TComPic* pic, const Double saoEncodingRate, const Double saoEncodingRateChroma)
{
  const Int picTempLayer = pic->getSlice(0)->getDepth();

  //decide sliceEnabled[compIdx]
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  for (Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    sliceEnabled[compIdx] = false;
  }

  for (Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    // reset flags & counters
    sliceEnabled[compIdx] = true;

    if (saoEncodingRate>0.0)
    {
      if (saoEncodingRateChroma>0.0)
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[compIdx][picTempLayer-1] > ((compIdx==COMPONENT_Y) ? saoEncodingRate : saoEncodingRateChroma)) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
      else
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[COMPONENT_Y][0] > saoEncodingRate) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
    }
  }
}

Int64 TEncSampleAdaptiveOffset::getDistortion(const Int channelBitDepth, Int typeIdc, Int typeAuxInfo, Int* invQuantOffset, SAOStatData& statData)
{
  Int64 dist        = 0;
  Int shift         = 2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth - 8);

  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        for (Int offsetIdx=0; offsetIdx<NUM_SAO_EO_CLASSES; offsetIdx++)
        {
          dist += estSaoDist( statData.count[offsetIdx], invQuantOffset[offsetIdx], statData.diff[offsetIdx], shift);
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        for (Int offsetIdx=typeAuxInfo; offsetIdx<typeAuxInfo+4; offsetIdx++)
        {
          Int bandIdx = offsetIdx % NUM_SAO_BO_CLASSES ;
          dist += estSaoDist( statData.count[bandIdx], invQuantOffset[bandIdx], statData.diff[bandIdx], shift);
        }
      }
      break;
    default:
      {
        printf("Not a supported type");
        assert(0);
        exit(-1);
      }
  }

  return dist;
}

inline Int64 TEncSampleAdaptiveOffset::estSaoDist(Int64 count, Int64 offset, Int64 diffSum, Int shift)
{
  return (( count*offset*offset-diffSum*offset*2 ) >> shift);
}


inline Int TEncSampleAdaptiveOffset::estIterOffset(Int typeIdx, Double lambda, Int offsetInput, Int64 count, Int64 diffSum, Int shift, Int bitIncrease, Int64& bestDist, Double& bestCost, Int offsetTh )
{
  Int iterOffset, tempOffset;
  Int64 tempDist, tempRate;
  Double tempCost, tempMinCost;
  Int offsetOutput = 0;
  iterOffset = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can be used to measure the exact rate here.
  tempMinCost = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = (typeIdx == SAO_TYPE_BO) ? (abs((Int)iterOffset)+2) : (abs((Int)iterOffset)+1);
    if (abs((Int)iterOffset)==offsetTh) //inclusive
    {
      tempRate --;
    }
    // Do the dequantization before distortion calculation
    tempOffset  = iterOffset << bitIncrease;
    tempDist    = estSaoDist( count, tempOffset, diffSum, shift);
    tempCost    = ((Double)tempDist + lambda * (Double) tempRate);
    if(tempCost < tempMinCost)
    {
      tempMinCost = tempCost;
      offsetOutput = iterOffset;
      bestDist = tempDist;
      bestCost = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset-1):(iterOffset+1);
  }
  return offsetOutput;
}

Void TEncSampleAdaptiveOffset::deriveOffsets(ComponentID compIdx, const Int channelBitDepth, Int typeIdc, SAOStatData& statData, Int* quantOffsets, Int& typeAuxInfo)
{
  Int bitDepth = channelBitDepth;
  Int shift    = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8);
  Int offsetTh = TComSampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);  //inclusive

  ::memset(quantOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  //derive initial offsets
  Int numClasses = (typeIdc == SAO_TYPE_BO)?((Int)NUM_SAO_BO_CLASSES):((Int)NUM_SAO_EO_CLASSES);
  for(Int classIdx=0; classIdx< numClasses; classIdx++)
  {
    if( (typeIdc != SAO_TYPE_BO) && (classIdx==SAO_CLASS_EO_PLAIN)  )
    {
      continue; //offset will be zero
    }

    if(statData.count[classIdx] == 0)
    {
      continue; //offset will be zero
    }

    quantOffsets[classIdx] = (Int) xRoundIbdi(bitDepth, (Double)( statData.diff[classIdx]<<(bitDepth-8))
                                                                  /
                                                          (Double)( statData.count[classIdx]<< m_offsetStepLog2[compIdx])
                                               );
    quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
  }

  // adjust offsets
  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        Int64 classDist;
        Double classCost;
        for(Int classIdx=0; classIdx<NUM_SAO_EO_CLASSES; classIdx++)
        {
          if(classIdx==SAO_CLASS_EO_FULL_VALLEY && quantOffsets[classIdx] < 0)
