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
