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
  rcDistParam.m_maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();
}

// Setting the Distortion Parameter for Inter (ME)
Void TComRdCost::setDistParam( const TComPattern* const pcPatternKey, const Pel* piRefY, Int iRefStride, DistParam& rcDistParam )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;

  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();
  rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  rcDistParam.m_maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();

  if (rcDistParam.iCols == 12)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD12];
  }
  else if (rcDistParam.iCols == 24)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD24];
  }
  else if (rcDistParam.iCols == 48)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD48];
  }

  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (subpel ME with step)
Void TComRdCost::setDistParam( const TComPattern* const pcPatternKey, const Pel* piRefY, Int iRefStride, Int iStep, DistParam& rcDistParam, Bool bHADME )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;

  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride * iStep;

  // set Step for interpolated buffer
  rcDistParam.iStep = iStep;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();

  rcDistParam.m_maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();

  // set distortion function
  if ( !bHADME )
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
    if (rcDistParam.iCols == 12)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS12];
    }
    else if (rcDistParam.iCols == 24)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS24];
    }
    else if (rcDistParam.iCols == 48)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS48];
    }
  }
  else
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_HADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  }

  // initialize
  rcDistParam.iSubShift  = 0;
}

Void TComRdCost::setDistParam( DistParam& rcDP, Int bitDepth, const Pel* p1, Int iStride1, const Pel* p2, Int iStride2, Int iWidth, Int iHeight, Bool bHadamard )
{
  rcDP.pOrg         = p1;
  rcDP.pCur         = p2;
  rcDP.iStrideOrg   = iStride1;
  rcDP.iStrideCur   = iStride2;
  rcDP.iCols        = iWidth;
  rcDP.iRows        = iHeight;
  rcDP.iStep        = 1;
  rcDP.iSubShift    = 0;
  rcDP.bitDepth     = bitDepth;
  rcDP.DistFunc     = m_afpDistortFunc[ ( bHadamard ? DF_HADS : DF_SADS ) + g_aucConvertToBit[ iWidth ] + 1 ];
  rcDP.m_maximumDistortionForEarlyExit = std::numeric_limits<Distortion>::max();
}

Distortion TComRdCost::calcHAD( Int bitDepth, const Pel* pi0, Int iStride0, const Pel* pi1, Int iStride1, Int iWidth, Int iHeight )
{
  Distortion uiSum = 0;
  Int x, y;

  if ( ( (iWidth % 8) == 0 ) && ( (iHeight % 8) == 0 ) )
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &pi0[x], &pi1[x], iStride0, iStride1, 1
#if VECTOR_CODING__DISTORTION_CALCULATIONS && (RExt__HIGH_BIT_DEPTH_SUPPORT==0)
          , bitDepth
#endif
          );
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else
  {
    assert ( ( (iWidth % 4) == 0 ) && ( (iHeight % 4) == 0 ) );

    for ( y=0; y<iHeight; y+= 4 )
    {
      for ( x=0; x<iWidth; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*4;
      pi1 += iStride1*4;
    }
  }

  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8) );
}

Distortion TComRdCost::getDistPart( Int bitDepth, const Pel* piCur, Int iCurStride,  const Pel* piOrg, Int iOrgStride, UInt uiBlkWidth, UInt uiBlkHeight, const ComponentID compID, DFunc eDFunc )
{
  DistParam cDtParam;
  setDistParam( uiBlkWidth, uiBlkHeight, eDFunc, cDtParam );
  cDtParam.pOrg       = piOrg;
  cDtParam.pCur       = piCur;
  cDtParam.iStrideOrg = iOrgStride;
  cDtParam.iStrideCur = iCurStride;
  cDtParam.iStep      = 1;

  cDtParam.bApplyWeight = false;
  cDtParam.compIdx      = MAX_NUM_COMPONENT; // just for assert: to be sure it was set before use
  cDtParam.bitDepth     = bitDepth;

  if (isChroma(compID))
  {
    return ((Distortion) (m_distortionWeight[compID] * cDtParam.DistFunc( &cDtParam )));
  }
  else
  {
    return cDtParam.DistFunc( &cDtParam );
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

#if VECTOR_CODING__DISTORTION_CALCULATIONS && (RExt__HIGH_BIT_DEPTH_SUPPORT==0)
inline Int simdSADLine4n16b( const Pel * piOrg , const Pel * piCur , Int nWidth )
{
  // internal bit-depth must be 12-bit or lower
  assert( !( nWidth & 0x03 ) );
  __m128i org , cur , abs , sum;
  sum = _mm_setzero_si128();
  for( Int n = 0 ; n < nWidth ; n += 4 )
  {
    org = _mm_loadl_epi64( ( __m128i* )( piOrg + n ) );
    cur = _mm_loadl_epi64( ( __m128i* )( piCur + n ) );
    abs = _mm_subs_epi16( _mm_max_epi16( org , cur )  , _mm_min_epi16( org , cur ) );
    sum = _mm_adds_epu16( abs , sum );
  }
  __m128i zero =  _mm_setzero_si128();
  sum = _mm_unpacklo_epi16( sum , zero );
  sum = _mm_add_epi32( sum , _mm_shuffle_epi32( sum , _MM_SHUFFLE( 2 , 3 , 0 , 1 ) ) );
  sum = _mm_add_epi32( sum , _mm_shuffle_epi32( sum , _MM_SHUFFLE( 1 , 0 , 3 , 2 ) ) );
  return( _mm_cvtsi128_si32( sum ) );
}

inline Int simdSADLine8n16b( const Pel * piOrg , const Pel * piCur , Int nWidth )
{
  // internal bit-depth must be 12-bit or lower
  assert( !( nWidth & 0x07 ) );
  __m128i org , cur , abs , sum;
  sum = _mm_setzero_si128();
  for( Int n = 0 ; n < nWidth ; n += 8 )
  {
    org = _mm_loadu_si128( ( __m128i* )( piOrg + n ) );
    cur = _mm_loadu_si128( ( __m128i* )( piCur + n ) );
    abs = _mm_subs_epi16( _mm_max_epi16( org , cur )  , _mm_min_epi16( org , cur ) );
    sum = _mm_adds_epu16( abs , sum );
  }
  __m128i zero =  _mm_setzero_si128();
  __m128i hi = _mm_unpackhi_epi16( sum , zero );
  __m128i lo = _mm_unpacklo_epi16( sum , zero );
  sum = _mm_add_epi32( lo , hi );
  sum = _mm_add_epi32( sum , _mm_shuffle_epi32( sum , _MM_SHUFFLE( 2 , 3 , 0 , 1 ) ) );
  sum = _mm_add_epi32( sum , _mm_shuffle_epi32( sum , _MM_SHUFFLE( 1 , 0 , 3 , 2 ) ) );
  return( _mm_cvtsi128_si32( sum ) );
}

inline Void simd8x8Transpose32b( __m128i * pBuffer )
{
  __m128 tmp[16];
  for( Int n = 0 ; n < 16 ; n++ )
  {
    tmp[n] = _mm_castsi128_ps( pBuffer[n] );
  }
  _MM_TRANSPOSE4_PS( tmp[0] , tmp[2] , tmp[4] , tmp[6] );
  _MM_TRANSPOSE4_PS( tmp[1] , tmp[3] , tmp[5] , tmp[7] );
  _MM_TRANSPOSE4_PS( tmp[8] , tmp[10] , tmp[12] , tmp[14] );
  _MM_TRANSPOSE4_PS( tmp[9] , tmp[11] , tmp[13] , tmp[15] );
  for( Int n = 0 ; n < 8 ; n += 2 )
  {
    pBuffer[n] = _mm_castps_si128( tmp[n] );
    pBuffer[n+1]  = _mm_castps_si128( tmp[n+8] );
    pBuffer[n+8] = _mm_castps_si128( tmp[n+1] );
    pBuffer[n+9]  = _mm_castps_si128( tmp[n+9] );
  }
}

#ifdef __GNUC__
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#if GCC_VERSION > 40600 && GCC_VERSION < 40700
__attribute__((optimize("no-tree-vrp")))
#endif
#endif
Void simd8x8HAD1D32b( __m128i * pInput , __m128i * pOutput )
{
  __m128i m1[8][2] , m2[8][2];

  m2[0][0] = _mm_add_epi32( pInput[0] ,pInput[8 ] );  m2[0][1] = _mm_add_epi32( pInput[1] ,pInput[9 ] );
  m2[1][0] = _mm_add_epi32( pInput[2] ,pInput[10] );  m2[1][1] = _mm_add_epi32( pInput[3] ,pInput[11] );
  m2[2][0] = _mm_add_epi32( pInput[4] ,pInput[12] );  m2[2][1] = _mm_add_epi32( pInput[5] ,pInput[13] );
  m2[3][0] = _mm_add_epi32( pInput[6] ,pInput[14] );  m2[3][1] = _mm_add_epi32( pInput[7] ,pInput[15] );
  m2[4][0] = _mm_sub_epi32( pInput[0] ,pInput[8 ] );  m2[4][1] = _mm_sub_epi32( pInput[1] ,pInput[9 ] );
  m2[5][0] = _mm_sub_epi32( pInput[2] ,pInput[10] );  m2[5][1] = _mm_sub_epi32( pInput[3] ,pInput[11] );
  m2[6][0] = _mm_sub_epi32( pInput[4] ,pInput[12] );  m2[6][1] = _mm_sub_epi32( pInput[5] ,pInput[13] );
  m2[7][0] = _mm_sub_epi32( pInput[6] ,pInput[14] );  m2[7][1] = _mm_sub_epi32( pInput[7] ,pInput[15] );

  m1[0][0] = _mm_add_epi32( m2[0][0] , m2[2][0] );  m1[0][1] = _mm_add_epi32( m2[0][1] , m2[2][1] );
  m1[1][0] = _mm_add_epi32( m2[1][0] , m2[3][0] );  m1[1][1] = _mm_add_epi32( m2[1][1] , m2[3][1] );
  m1[2][0] = _mm_sub_epi32( m2[0][0] , m2[2][0] );  m1[2][1] = _mm_sub_epi32( m2[0][1] , m2[2][1] );
  m1[3][0] = _mm_sub_epi32( m2[1][0] , m2[3][0] );  m1[3][1] = _mm_sub_epi32( m2[1][1] , m2[3][1] );
  m1[4][0] = _mm_add_epi32( m2[4][0] , m2[6][0] );  m1[4][1] = _mm_add_epi32( m2[4][1] , m2[6][1] );
  m1[5][0] = _mm_add_epi32( m2[5][0] , m2[7][0] );  m1[5][1] = _mm_add_epi32( m2[5][1] , m2[7][1] );
  m1[6][0] = _mm_sub_epi32( m2[4][0] , m2[6][0] );  m1[6][1] = _mm_sub_epi32( m2[4][1] , m2[6][1] );
  m1[7][0] = _mm_sub_epi32( m2[5][0] , m2[7][0] );  m1[7][1] = _mm_sub_epi32( m2[5][1] , m2[7][1] );

  pInput[0 ] = _mm_add_epi32( m1[0][0] , m1[1][0] );  pInput[1 ] = _mm_add_epi32( m1[0][1] , m1[1][1] );
  pInput[2 ] = _mm_sub_epi32( m1[0][0] , m1[1][0] );  pInput[3 ] = _mm_sub_epi32( m1[0][1] , m1[1][1] );
  pInput[4 ] = _mm_add_epi32( m1[2][0] , m1[3][0] );  pInput[5 ] = _mm_add_epi32( m1[2][1] , m1[3][1] );
  pInput[6 ] = _mm_sub_epi32( m1[2][0] , m1[3][0] );  pInput[7 ] = _mm_sub_epi32( m1[2][1] , m1[3][1] );
  pInput[8 ] = _mm_add_epi32( m1[4][0] , m1[5][0] );  pInput[9 ] = _mm_add_epi32( m1[4][1] , m1[5][1] );
  pInput[10] = _mm_sub_epi32( m1[4][0] , m1[5][0] );  pInput[11] = _mm_sub_epi32( m1[4][1] , m1[5][1] );
  pInput[12] = _mm_add_epi32( m1[6][0] , m1[7][0] );  pInput[13] = _mm_add_epi32( m1[6][1] , m1[7][1] );
  pInput[14] = _mm_sub_epi32( m1[6][0] , m1[7][0] );  pInput[15] = _mm_sub_epi32( m1[6][1] , m1[7][1] );
}

inline __m128i simdAbs32b( __m128i m )
{
  const __m128i zero = _mm_setzero_si128();
  __m128i tmp = _mm_sub_epi32( zero , m );
  __m128i mask = _mm_cmpgt_epi32( m , tmp );
  return( _mm_or_si128( _mm_and_si128( mask , m ) , _mm_andnot_si128( mask , tmp ) ) );
}

UInt simdHADs8x8( const Pel * piOrg, const Pel * piCur, Int iStrideOrg, Int iStrideCur )
{
  __m128i mmDiff[8][2];
  __m128i mmZero = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ , piOrg += iStrideOrg , piCur += iStrideCur )
  {
    __m128i diff = _mm_sub_epi16( _mm_loadu_si128( ( __m128i* )piOrg ) , _mm_loadu_si128( ( __m128i* )piCur ) );
    // sign extension
    __m128i mask = _mm_cmplt_epi16( diff , mmZero );
    mmDiff[n][0] = _mm_unpacklo_epi16( diff , mask );
    mmDiff[n][1] = _mm_unpackhi_epi16( diff , mask );
  }

  // transpose
  simd8x8Transpose32b( &mmDiff[0][0] );

  // horizontal
  simd8x8HAD1D32b( &mmDiff[0][0] , &mmDiff[0][0] );

  // transpose
  simd8x8Transpose32b( &mmDiff[0][0] );

  // vertical
  simd8x8HAD1D32b( &mmDiff[0][0] , &mmDiff[0][0] );

  __m128i mmSum = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ )
  {
