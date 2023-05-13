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

/** \file     TComRdCost.cpp
    \brief    RD cost computation class
*/

#include <math.h>
#include <assert.h>
#include <limits>
#include "TComRom.h"
#include "TComRdCost.h"

#if VECTOR_CODING__DISTORTION_CALCULATIONS && (RExt__HIGH_BIT_DEPTH_SUPPORT==0)
#include <emmintrin.h>
#include <xmmintrin.h>
#endif

//! \ingroup TLibCommon
//! \{

TComRdCost::TComRdCost()
{
  init();
}

TComRdCost::~TComRdCost()
{
}

// Calculate RD functions
Double TComRdCost::calcRdCost( Double numBits, Distortion distortion, DFunc eDFunc )
{
  Double lambda = 1.0;

  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
      lambda = m_dLambdaMotionSAD[0]; // 0 is valid, because for lossless blocks, the cost equation is modified to compensate.
      break;
    case DF_DEFAULT:
      lambda = m_dLambda;
      break;
    case DF_SSE_FRAME:
      lambda = m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }

  if (eDFunc == DF_SAD)
  {
    if (m_costMode != COST_STANDARD_LOSSY)
    {
      return ((distortion * 65536.0) / lambda) + numBits; // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
    }
    else
    {
      return distortion + (((numBits * lambda) ) / 65536.0);
    }
  }
  else
  {
    if (m_costMode != COST_STANDARD_LOSSY)
    {
      return (distortion / lambda) + numBits; // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
    }
    else
    {
      return distortion + (numBits * lambda);
    }
  }
}

Void TComRdCost::setLambda( Double dLambda, const BitDepths &bitDepths )
{
  m_dLambda           = dLambda;
  m_sqrtLambda        = sqrt(m_dLambda);
  m_dLambdaMotionSAD[0] = 65536.0 * m_sqrtLambda;
  m_dLambdaMotionSSE[0] = 65536.0 * m_dLambda;
#if FULL_NBIT
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0));
#else
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (bitDepths.recon[CHANNEL_TYPE_LUMA] - 8)) / 3.0));
#endif
  m_dLambdaMotionSAD[1] = 65536.0 * sqrt(dLambda);
  m_dLambdaMotionSSE[1] = 65536.0 * dLambda;
}


// Initalize Function Pointer by [eDFunc]
Void TComRdCost::init()
{
  m_afpDistortFunc[DF_DEFAULT] = NULL;                  // for DF_DEFAULT

  m_afpDistortFunc[DF_SSE    ] = TComRdCost::xGetSSE;
  m_afpDistortFunc[DF_SSE4   ] = TComRdCost::xGetSSE4;
  m_afpDistortFunc[DF_SSE8   ] = TComRdCost::xGetSSE8;
  m_afpDistortFunc[DF_SSE16  ] = TComRdCost::xGetSSE16;
  m_afpDistortFunc[DF_SSE32  ] = TComRdCost::xGetSSE32;
  m_afpDistortFunc[DF_SSE64  ] = TComRdCost::xGetSSE64;
  m_afpDistortFunc[DF_SSE16N ] = TComRdCost::xGetSSE16N;

  m_afpDistortFunc[DF_SAD    ] = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SAD4   ] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SAD8   ] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SAD16  ] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SAD32  ] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SAD64  ] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SAD16N ] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SADS   ] = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SADS4  ] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SADS8  ] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SADS16 ] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SADS32 ] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SADS64 ] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SADS16N] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SAD12  ] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SAD24  ] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SAD48  ] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[DF_SADS12 ] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SADS24 ] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SADS48 ] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[DF_HADS   ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS4  ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS8  ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS16 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS32 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS64 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS16N] = TComRdCost::xGetHADs;

  m_costMode                   = COST_STANDARD_LOSSY;

  m_motionLambda               = 0;
  m_iCostScale                 = 0;
}

// Static member function
UInt TComRdCost::xGetExpGolombNumberOfBits( Int iVal )
{
  assert(iVal != std::numeric_limits<Int>::min());
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (UInt(-iVal)<<1)+1: UInt(iVal<<1);

  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }

  return uiLength;
}

Void TComRdCost::setDistParam( UInt uiBlkWidth, UInt uiBlkHeight, DFunc eDFunc, DistParam& rcDistParam )
{
  // set Block Width / Height
  rcDistParam.iCols    = uiBlkWidth;
  rcDistParam.iRows    = uiBlkHeight;
  rcDistParam.DistFunc = m_afpDistortFunc[eDFunc + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];

  // initialize
  rcDistParam.iSubShift  = 0;
