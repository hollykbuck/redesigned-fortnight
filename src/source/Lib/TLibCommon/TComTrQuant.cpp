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

/** \file     TComTrQuant.cpp
    \brief    transform and quantization class
*/

#include <stdlib.h>
#include <math.h>
#include <limits>
#include <memory.h>
#include "TComTrQuant.h"
#include "TComPic.h"
#include "ContextTables.h"
#include "TComTU.h"
#include "Debug.h"

typedef struct
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
} coeffGroupRDStats;

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define RDOQ_CHROMA                 1           ///< use of RDOQ in chroma


// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam(const Int           qpy,
                 const ChannelType   chType,
                 const Int           qpBdOffset,
                 const Int           chromaQPOffset,
                 const ChromaFormat  chFmt )
{
  Int baseQp;

  if(isLuma(chType))
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }
  }

  Qp =baseQp;
  per=baseQp/6;
  rem=baseQp%6;
}

QpParam::QpParam(const TComDataCU &cu, const ComponentID compID)
{
  Int chromaQpOffset = 0;

  if (isChroma(compID))
  {
    chromaQpOffset += cu.getSlice()->getPPS()->getQpOffset(compID);
    chromaQpOffset += cu.getSlice()->getSliceChromaQpDelta(compID);

    chromaQpOffset += cu.getSlice()->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEntry(cu.getChromaQpAdj(0)).u.offset[Int(compID)-1];
  }

  *this = QpParam(cu.getQP( 0 ),
                  toChannelType(compID),
                  cu.getSlice()->getSPS()->getQpBDOffset(toChannelType(compID)),
                  chromaQpOffset,
                  cu.getPic()->getChromaFormat());
}


// ====================================================================================================================
// TComTrQuant class member functions
// ====================================================================================================================

TComTrQuant::TComTrQuant()
{
  // allocate temporary buffers
  m_plTempCoeff  = new TCoeff[ MAX_CU_SIZE*MAX_CU_SIZE ];

  // allocate bit estimation class  (for RDOQ)
  m_pcEstBitsSbac = new estBitsSbacStruct;
  initScalingList();
}

TComTrQuant::~TComTrQuant()
{
  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    delete [] m_plTempCoeff;
    m_plTempCoeff = NULL;
  }

  // delete bit estimation class
  if ( m_pcEstBitsSbac )
  {
    delete m_pcEstBitsSbac;
  }
  destroyScalingList();
}

#if ADAPTIVE_QP_SELECTION
Void TComTrQuant::storeSliceQpNext(TComSlice* pcSlice)
{
  // NOTE: does this work with negative QPs or when some blocks are transquant-bypass enabled?

  Int qpBase = pcSlice->getSliceQpBase();
  Int sliceQpused = pcSlice->getSliceQp();
  Int sliceQpnext;
  Double alpha = qpBase < 17 ? 0.5 : 1;

  Int cnt=0;
  for(Int u=1; u<=LEVEL_RANGE; u++)
  {
    cnt += m_sliceNsamples[u] ;
  }

  if( !m_useRDOQ )
  {
    sliceQpused = qpBase;
    alpha = 0.5;
  }

  if( cnt > 120 )
  {
    Double sum = 0;
    Int k = 0;
    for(Int u=1; u<LEVEL_RANGE; u++)
    {
      sum += u*m_sliceSumC[u];
      k += u*u*m_sliceNsamples[u];
    }

    Int v;
    Double q[MAX_QP+1] ;
    for(v=0; v<=MAX_QP; v++)
    {
      q[v] = (Double)(g_invQuantScales[v%6] * (1<<(v/6)))/64 ;
    }

    Double qnext = sum/k * q[sliceQpused] / (1<<ARL_C_PRECISION);

    for(v=0; v<MAX_QP; v++)
    {
      if(qnext < alpha * q[v] + (1 - alpha) * q[v+1] )
      {
        break;
      }
    }
    sliceQpnext = Clip3(sliceQpused - 3, sliceQpused + 3, v);
  }
  else
  {
    sliceQpnext = sliceQpused;
  }

  m_qpDelta[qpBase] = sliceQpnext - qpBase;
}

Void TComTrQuant::initSliceQpDelta()
{
  for(Int qp=0; qp<=MAX_QP; qp++)
  {
    m_qpDelta[qp] = qp < 17 ? 0 : 1;
  }
}

Void TComTrQuant::clearSliceARLCnt()
{
  memset(m_sliceSumC, 0, sizeof(Double)*(LEVEL_RANGE+1));
  memset(m_sliceNsamples, 0, sizeof(Int)*(LEVEL_RANGE+1));
}
#endif



#if MATRIX_MULT
/** NxN forward transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param block pointer to input data (residual)
 *  \param coeff pointer to output data (transform coefficients)
 *  \param uiStride stride of input data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void xTr(Int bitDepth, Pel *block, TCoeff *coeff, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;
  UInt uiLog2TrSize = g_aucConvertToBit[ uiTrSize ] + 2;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[TRANSFORM_FORWARD][0] : g_aiT4[TRANSFORM_FORWARD][0]);
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[TRANSFORM_FORWARD][0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[TRANSFORM_FORWARD][0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[TRANSFORM_FORWARD][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = (uiLog2TrSize +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = uiLog2TrSize + TRANSFORM_MATRIX_SHIFT;
  const Int add_1st = (shift_1st>0) ? (1<<(shift_1st-1)) : 0;
  const Int add_2nd = 1<<(shift_2nd-1);

  /* Horizontal transform */

  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*block[j*uiStride+k];
      }
      tmp[i*uiTrSize+j] = (iSum + add_1st)>>shift_1st;
    }
  }

  /* Vertical transform */
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*tmp[j*uiTrSize+k];
      }
      coeff[i*uiTrSize+j] = (iSum + add_2nd)>>shift_2nd;
    }
  }
}
