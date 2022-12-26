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

/** NxN inverse transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param coeff pointer to input data (transform coefficients)
 *  \param block pointer to output data (residual)
 *  \param uiStride stride of output data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void xITr(Int bitDepth, TCoeff *coeff, Pel *block, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[TRANSFORM_INVERSE][0] : g_aiT4[TRANSFORM_INVERSE][0]);
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[TRANSFORM_INVERSE][0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[TRANSFORM_INVERSE][0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[TRANSFORM_INVERSE][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];

  const Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  const Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;
  assert(shift_2nd>=0);
  const Int add_1st = 1<<(shift_1st-1);
  const Int add_2nd = (shift_2nd>0) ? (1<<(shift_2nd-1)) : 0;

  /* Horizontal transform */
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[k*uiTrSize+i]*coeff[k*uiTrSize+j];
      }

      // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
      tmp[i*uiTrSize+j] = Clip3<TCoeff>(clipMinimum, clipMaximum, (iSum + add_1st)>>shift_1st);
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
        iSum += iT[k*uiTrSize+j]*tmp[i*uiTrSize+k];
      }

      block[i*uiStride+j] = Clip3<TCoeff>(std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), (iSum + add_2nd)>>shift_2nd);
    }
  }
}

#endif //MATRIX_MULT


/** 4x4 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly4(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0]      = (g_aiT4[TRANSFORM_FORWARD][0][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[TRANSFORM_FORWARD][2][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][2][1]*E[1] + add)>>shift;
    dst[line]   = (g_aiT4[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][3][1]*O[1] + add)>>shift;

    src += 4;
    dst ++;
  }
}

// Fast DST Algorithm. Full matrix multiplication for DST and Fast DST algorithm
// give identical results
Void fastForwardDst(TCoeff *block, TCoeff *coeff, Int shift)  // input block, output coeff
{
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = block[4*i+0];
    c[1] = block[4*i+1];
    c[2] = block[4*i+2];
    c[3] = block[4*i+3];

    for (Int row = 0; row < 4; row++)
    {
      TCoeff result = 0;
      for (Int column = 0; column < 4; column++)
      {
        result += c[column] * g_as_DST_MAT_4[TRANSFORM_FORWARD][row][column]; // use the defined matrix, rather than hard-wired numbers
      }

      coeff[(row * 4) + i] = rightShift((result + rnd_factor), shift);
    }
  }
}

Void fastInverseDst(TCoeff *tmp, TCoeff *block, Int shift, const TCoeff outputMinimum, const TCoeff outputMaximum)  // input tmp, output block
{
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = tmp[   i];
    c[1] = tmp[4 +i];
    c[2] = tmp[8 +i];
    c[3] = tmp[12+i];

    for (Int column = 0; column < 4; column++)
    {
      TCoeff &result = block[(i * 4) + column];

      result = 0;
      for (Int row = 0; row < 4; row++)
      {
        result += c[row] * g_as_DST_MAT_4[TRANSFORM_INVERSE][row][column]; // use the defined matrix, rather than hard-wired numbers
      }

      result = Clip3( outputMinimum, outputMaximum, rightShift((result + rnd_factor), shift));
    }
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse4(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = g_aiT4[TRANSFORM_INVERSE][1][0]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][0]*src[3*line];
    O[1] = g_aiT4[TRANSFORM_INVERSE][1][1]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][1]*src[3*line];
    E[0] = g_aiT4[TRANSFORM_INVERSE][0][0]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][0]*src[2*line];
    E[1] = g_aiT4[TRANSFORM_INVERSE][0][1]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][1]*src[2*line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( outputMinimum, outputMaximum, (E[0] + O[0] + add)>>shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, (E[1] + O[1] + add)>>shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, (E[1] - O[1] + add)>>shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, (E[0] - O[0] + add)>>shift );

    src   ++;
    dst += 4;
  }
}

/** 8x8 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly8(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<4;k++)
    {
      E[k] = src[k] + src[7-k];
      O[k] = src[k] - src[7-k];
    }
    /* EE and EO */
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0]      = (g_aiT8[TRANSFORM_FORWARD][0][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[TRANSFORM_FORWARD][4][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][4][1]*EE[1] + add)>>shift;
    dst[2*line] = (g_aiT8[TRANSFORM_FORWARD][2][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[TRANSFORM_FORWARD][6][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][6][1]*EO[1] + add)>>shift;

    dst[line]   = (g_aiT8[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][1][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][1][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][3][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][3][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[TRANSFORM_FORWARD][5][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][5][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][5][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[TRANSFORM_FORWARD][7][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][7][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][7][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][7][3]*O[3] + add)>>shift;

    src += 8;
    dst ++;
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse8(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[TRANSFORM_INVERSE][ 1][k]*src[line]   + g_aiT8[TRANSFORM_INVERSE][ 3][k]*src[3*line] +
             g_aiT8[TRANSFORM_INVERSE][ 5][k]*src[5*line] + g_aiT8[TRANSFORM_INVERSE][ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[TRANSFORM_INVERSE][2][0]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][0]*src[ 6*line ];
    EO[1] = g_aiT8[TRANSFORM_INVERSE][2][1]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][1]*src[ 6*line ];
    EE[0] = g_aiT8[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][0]*src[ 4*line ];
    EE[1] = g_aiT8[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][1]*src[ 4*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (k=0;k<4;k++)
    {
      dst[ k   ] = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[ k+4 ] = Clip3( outputMinimum, outputMaximum, (E[3-k] - O[3-k] + add)>>shift );
    }
    src ++;
    dst += 8;
  }
}

/** 16x16 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly16(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<8;k++)
    {
      E[k] = src[k] + src[15-k];
      O[k] = src[k] - src[15-k];
    }
    /* EE and EO */
    for (k=0;k<4;k++)
    {
      EE[k] = E[k] + E[7-k];
      EO[k] = E[k] - E[7-k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0      ] = (g_aiT16[TRANSFORM_FORWARD][ 0][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 0][1]*EEE[1] + add)>>shift;
    dst[ 8*line ] = (g_aiT16[TRANSFORM_FORWARD][ 8][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 8][1]*EEE[1] + add)>>shift;
    dst[ 4*line ] = (g_aiT16[TRANSFORM_FORWARD][ 4][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][ 4][1]*EEO[1] + add)>>shift;
    dst[ 12*line] = (g_aiT16[TRANSFORM_FORWARD][12][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][12][1]*EEO[1] + add)>>shift;

    for (k=2;k<16;k+=4)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*EO[3] + add)>>shift;
    }

    for (k=1;k<16;k+=2)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*O[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*O[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*O[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*O[3] +
                       g_aiT16[TRANSFORM_FORWARD][k][4]*O[4] + g_aiT16[TRANSFORM_FORWARD][k][5]*O[5] +
                       g_aiT16[TRANSFORM_FORWARD][k][6]*O[6] + g_aiT16[TRANSFORM_FORWARD][k][7]*O[7] + add)>>shift;
    }

    src += 16;
    dst ++;

  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
 *  \param src            input data (transform coefficients)
 *  \param dst            output data (residual)
 *  \param shift          specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse16(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<8;k++)
    {
      O[k] = g_aiT16[TRANSFORM_INVERSE][ 1][k]*src[ line]   + g_aiT16[TRANSFORM_INVERSE][ 3][k]*src[ 3*line] +
             g_aiT16[TRANSFORM_INVERSE][ 5][k]*src[ 5*line] + g_aiT16[TRANSFORM_INVERSE][ 7][k]*src[ 7*line] +
             g_aiT16[TRANSFORM_INVERSE][ 9][k]*src[ 9*line] + g_aiT16[TRANSFORM_INVERSE][11][k]*src[11*line] +
             g_aiT16[TRANSFORM_INVERSE][13][k]*src[13*line] + g_aiT16[TRANSFORM_INVERSE][15][k]*src[15*line];
    }
    for (k=0;k<4;k++)
    {
      EO[k] = g_aiT16[TRANSFORM_INVERSE][ 2][k]*src[ 2*line] + g_aiT16[TRANSFORM_INVERSE][ 6][k]*src[ 6*line] +
              g_aiT16[TRANSFORM_INVERSE][10][k]*src[10*line] + g_aiT16[TRANSFORM_INVERSE][14][k]*src[14*line];
    }
    EEO[0] = g_aiT16[TRANSFORM_INVERSE][4][0]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][0]*src[ 12*line ];
    EEE[0] = g_aiT16[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][0]*src[ 8*line  ];
    EEO[1] = g_aiT16[TRANSFORM_INVERSE][4][1]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][1]*src[ 12*line ];
    EEE[1] = g_aiT16[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][1]*src[ 8*line  ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k=0;k<2;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+2] = EEE[1-k] - EEO[1-k];
    }
    for (k=0;k<4;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+4] = EE[3-k] - EO[3-k];
    }
    for (k=0;k<8;k++)
    {
      dst[k]   = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+8] = Clip3( outputMinimum, outputMaximum, (E[7-k] - O[7-k] + add)>>shift );
    }
    src ++;
    dst += 16;
  }
}

/** 32x32 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly32(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<16;k++)
    {
      E[k] = src[k] + src[31-k];
      O[k] = src[k] - src[31-k];
    }
    /* EE and EO */
    for (k=0;k<8;k++)
    {
      EE[k] = E[k] + E[15-k];
      EO[k] = E[k] - E[15-k];
    }
    /* EEE and EEO */
    for (k=0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7-k];
      EEO[k] = EE[k] - EE[7-k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[ 0       ] = (g_aiT32[TRANSFORM_FORWARD][ 0][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][ 0][1]*EEEE[1] + add)>>shift;
    dst[ 16*line ] = (g_aiT32[TRANSFORM_FORWARD][16][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][16][1]*EEEE[1] + add)>>shift;
    dst[ 8*line  ] = (g_aiT32[TRANSFORM_FORWARD][ 8][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][ 8][1]*EEEO[1] + add)>>shift;
    dst[ 24*line ] = (g_aiT32[TRANSFORM_FORWARD][24][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][24][1]*EEEO[1] + add)>>shift;
    for (k=4;k<32;k+=8)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EEO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EEO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EEO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EEO[3] + add)>>shift;
    }
    for (k=2;k<32;k+=4)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EO[3] +
                       g_aiT32[TRANSFORM_FORWARD][k][4]*EO[4] + g_aiT32[TRANSFORM_FORWARD][k][5]*EO[5] +
                       g_aiT32[TRANSFORM_FORWARD][k][6]*EO[6] + g_aiT32[TRANSFORM_FORWARD][k][7]*EO[7] + add)>>shift;
    }
    for (k=1;k<32;k+=2)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][ 0]*O[ 0] + g_aiT32[TRANSFORM_FORWARD][k][ 1]*O[ 1] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 2]*O[ 2] + g_aiT32[TRANSFORM_FORWARD][k][ 3]*O[ 3] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 4]*O[ 4] + g_aiT32[TRANSFORM_FORWARD][k][ 5]*O[ 5] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 6]*O[ 6] + g_aiT32[TRANSFORM_FORWARD][k][ 7]*O[ 7] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 8]*O[ 8] + g_aiT32[TRANSFORM_FORWARD][k][ 9]*O[ 9] +
                       g_aiT32[TRANSFORM_FORWARD][k][10]*O[10] + g_aiT32[TRANSFORM_FORWARD][k][11]*O[11] +
                       g_aiT32[TRANSFORM_FORWARD][k][12]*O[12] + g_aiT32[TRANSFORM_FORWARD][k][13]*O[13] +
                       g_aiT32[TRANSFORM_FORWARD][k][14]*O[14] + g_aiT32[TRANSFORM_FORWARD][k][15]*O[15] + add)>>shift;
    }

    src += 32;
    dst ++;
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse32(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<16;k++)
    {
      O[k] = g_aiT32[TRANSFORM_INVERSE][ 1][k]*src[ line    ] + g_aiT32[TRANSFORM_INVERSE][ 3][k]*src[ 3*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 5][k]*src[ 5*line  ] + g_aiT32[TRANSFORM_INVERSE][ 7][k]*src[ 7*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 9][k]*src[ 9*line  ] + g_aiT32[TRANSFORM_INVERSE][11][k]*src[ 11*line ] +
             g_aiT32[TRANSFORM_INVERSE][13][k]*src[ 13*line ] + g_aiT32[TRANSFORM_INVERSE][15][k]*src[ 15*line ] +
             g_aiT32[TRANSFORM_INVERSE][17][k]*src[ 17*line ] + g_aiT32[TRANSFORM_INVERSE][19][k]*src[ 19*line ] +
             g_aiT32[TRANSFORM_INVERSE][21][k]*src[ 21*line ] + g_aiT32[TRANSFORM_INVERSE][23][k]*src[ 23*line ] +
             g_aiT32[TRANSFORM_INVERSE][25][k]*src[ 25*line ] + g_aiT32[TRANSFORM_INVERSE][27][k]*src[ 27*line ] +
             g_aiT32[TRANSFORM_INVERSE][29][k]*src[ 29*line ] + g_aiT32[TRANSFORM_INVERSE][31][k]*src[ 31*line ];
    }
    for (k=0;k<8;k++)
    {
      EO[k] = g_aiT32[TRANSFORM_INVERSE][ 2][k]*src[ 2*line  ] + g_aiT32[TRANSFORM_INVERSE][ 6][k]*src[ 6*line  ] +
              g_aiT32[TRANSFORM_INVERSE][10][k]*src[ 10*line ] + g_aiT32[TRANSFORM_INVERSE][14][k]*src[ 14*line ] +
              g_aiT32[TRANSFORM_INVERSE][18][k]*src[ 18*line ] + g_aiT32[TRANSFORM_INVERSE][22][k]*src[ 22*line ] +
              g_aiT32[TRANSFORM_INVERSE][26][k]*src[ 26*line ] + g_aiT32[TRANSFORM_INVERSE][30][k]*src[ 30*line ];
    }
    for (k=0;k<4;k++)
    {
      EEO[k] = g_aiT32[TRANSFORM_INVERSE][ 4][k]*src[  4*line ] + g_aiT32[TRANSFORM_INVERSE][12][k]*src[ 12*line ] +
               g_aiT32[TRANSFORM_INVERSE][20][k]*src[ 20*line ] + g_aiT32[TRANSFORM_INVERSE][28][k]*src[ 28*line ];
    }
    EEEO[0] = g_aiT32[TRANSFORM_INVERSE][8][0]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][0]*src[ 24*line ];
    EEEO[1] = g_aiT32[TRANSFORM_INVERSE][8][1]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][1]*src[ 24*line ];
    EEEE[0] = g_aiT32[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][0]*src[ 16*line ];
    EEEE[1] = g_aiT32[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][1]*src[ 16*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k=0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+4] = EEE[3-k] - EEO[3-k];
    }
    for (k=0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+8] = EE[7-k] - EO[7-k];
    }
    for (k=0;k<16;k++)
    {
      dst[k]    = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+16] = Clip3( outputMinimum, outputMaximum, (E[15-k] - O[15-k] + add)>>shift );
    }
    src ++;
    dst += 32;
  }
}

/** MxN forward transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param block                 [in]  residual block
*  \param coeff                 [out] transform coefficients
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]

*/
Void xTrMxN(Int bitDepth, TCoeff *block, TCoeff *coeff, Int iWidth, Int iHeight, Bool useDST, const Int maxLog2TrDynamicRange)
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = ((g_aucConvertToBit[iWidth] + 2) +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = (g_aucConvertToBit[iHeight] + 2) + TRANSFORM_MATRIX_SHIFT;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[ MAX_TU_SIZE * MAX_TU_SIZE ];

  switch (iWidth)
  {
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
           fastForwardDst( block, tmp, shift_1st );
        }
        else
        {
          partialButterfly4 ( block, tmp, shift_1st, iHeight );
        }
      }
      break;

    case 8:     partialButterfly8 ( block, tmp, shift_1st, iHeight );  break;
    case 16:    partialButterfly16( block, tmp, shift_1st, iHeight );  break;
    case 32:    partialButterfly32( block, tmp, shift_1st, iHeight );  break;
    default:
      assert(0); exit (1); break;
  }

  switch (iHeight)
  {
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDst( tmp, coeff, shift_2nd );
        }
        else
        {
          partialButterfly4 ( tmp, coeff, shift_2nd, iWidth );
        }
      }
      break;

    case 8:     partialButterfly8 ( tmp, coeff, shift_2nd, iWidth );    break;
    case 16:    partialButterfly16( tmp, coeff, shift_2nd, iWidth );    break;
    case 32:    partialButterfly32( tmp, coeff, shift_2nd, iWidth );    break;
    default:
      assert(0); exit (1); break;
  }
}


/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param block                 [out] residual block
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*/
Void xITrMxN(Int bitDepth, TCoeff *coeff, TCoeff *block, Int iWidth, Int iHeight, Bool useDST, const Int maxLog2TrDynamicRange)
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];

  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];

  switch (iHeight)
  {
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( coeff, tmp, shift_1st, clipMinimum, clipMaximum);
        }
        else
        {
          partialButterflyInverse4 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum);
        }
      }
      break;

    case  8: partialButterflyInverse8 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 16: partialButterflyInverse16( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 32: partialButterflyInverse32( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;

    default:
      assert(0); exit (1); break;
  }

  switch (iWidth)
  {
    // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( tmp, block, shift_2nd, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
        }
        else
        {
          partialButterflyInverse4 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max());
        }
      }
      break;

    case  8: partialButterflyInverse8 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 16: partialButterflyInverse16( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 32: partialButterflyInverse32( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;

    default:
      assert(0); exit (1); break;
  }
}


// To minimize the distortion only. No rate is considered.
Void TComTrQuant::signBitHidingHDQ( TCoeff* pQCoef, TCoeff* pCoef, TCoeff* deltaU, const TUEntropyCodingParameters &codingParameters, const Int maxLog2TrDynamicRange )
{
  const UInt width     = codingParameters.widthInGroups  << MLS_CG_LOG2_WIDTH;
  const UInt height    = codingParameters.heightInGroups << MLS_CG_LOG2_HEIGHT;
  const UInt groupSize = 1 << MLS_CG_SIZE;

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> MLS_CG_SIZE; subSet >= 0; subSet-- )
  {
    Int  subPos = subSet << MLS_CG_SIZE;
    Int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += Int(pQCoef[ codingParameters.scan[ n + subPos ]]);
    }

    if(lastNZPosInCG>=0 && lastCG==-1)
    {
      lastCG = 1 ;
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      UInt signbit = (pQCoef[codingParameters.scan[subPos+firstNZPosInCG]]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        Int minPos =-1, finalChange=0, curChange=0;

        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )
        {
          UInt blkPos   = codingParameters.scan[ n+subPos ];
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos];
              curChange=1 ;
            }
            else
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = deltaU[blkPos];
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              UInt thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ;
        }
        else
        {
          pQCoef[minPos] -= finalChange ;
        }
      } // Hide
    }
    if(lastCG==1)
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}


Void TComTrQuant::xQuant(       TComTU       &rTu,
                                TCoeff      * pSrc,
                                TCoeff      * pDes,
#if ADAPTIVE_QP_SELECTION
                                TCoeff      *pArlDes,
#endif
                                TCoeff       &uiAbsSum,
                          const ComponentID   compID,
                          const QpParam      &cQP )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));

  TCoeff* piCoef    = pSrc;
  TCoeff* piQCoef   = pDes;
#if ADAPTIVE_QP_SELECTION
  TCoeff* piArlCCoef = pArlDes;
#endif

  const Bool useTransformSkip      = pcCU->getTransformSkip(uiAbsPartIdx, compID);
  const Int  maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));

  Bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;
  if ( useRDOQ && (isLuma(compID) || RDOQ_CHROMA) )
  {
    if ( !m_useSelectiveRDOQ || xNeedRDOQ( rTu, piCoef, compID, cQP ) )
    {
#if ADAPTIVE_QP_SELECTION
      xRateDistOptQuant( rTu, piCoef, pDes, pArlDes, uiAbsSum, compID, cQP );
#else
      xRateDistOptQuant( rTu, piCoef, pDes, uiAbsSum, compID, cQP );
#endif
    }
    else
    {
      memset( pDes, 0, sizeof( TCoeff ) * uiWidth *uiHeight );
      uiAbsSum = 0;
    }
  }
  else
  {
    TUEntropyCodingParameters codingParameters;
    getTUEntropyCodingParameters(codingParameters, rTu, compID);

    const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
    const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

    TCoeff deltaU[MAX_TU_SIZE * MAX_TU_SIZE];

    const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

    Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
    assert(scalingListType < SCALING_LIST_NUM);
    Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrSize-2);

    const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));
    const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */

    // Represents scaling through forward transform
    Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);
    if (useTransformSkip && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
    {
      iTransformShift = std::max<Int>(0, iTransformShift);
    }

    const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
    // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

#if ADAPTIVE_QP_SELECTION
    Int iQBitsC = MAX_INT;
    Int iAddC   = MAX_INT;

    if (m_bUseAdaptQpSelect)
    {
      iQBitsC = iQBits - ARL_C_PRECISION;
      iAddC   = 1 << (iQBitsC-1);
    }
#endif

    const Int iAdd   = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);
    const Int qBits8 = iQBits - 8;

    for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )
    {
      const TCoeff iLevel   = piCoef[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);

      const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlCCoef[uiBlockPos] = (TCoeff)((tmpLevel + iAddC ) >> iQBitsC);
      }
#endif

      const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel - (quantisedMagnitude<<iQBits) )>> qBits8);

      uiAbsSum += quantisedMagnitude;
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

      piQCoef[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    } // for n

    if( pcCU->getSlice()->getPPS()->getSignDataHidingEnabledFlag() )
    {
      if(uiAbsSum >= 2) //this prevents TUs with only one coefficient of value 1 from being tested
      {
        signBitHidingHDQ( piQCoef, piCoef, deltaU, codingParameters, maxLog2TrDynamicRange ) ;
      }
    }
  } //if RDOQ
  //return;
}

Bool TComTrQuant::xNeedRDOQ( TComTU &rTu, TCoeff * pSrc, const ComponentID compID, const QpParam &cQP )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));

  TCoeff* piCoef    = pSrc;

  const Bool useTransformSkip      = pcCU->getTransformSkip(uiAbsPartIdx, compID);
  const Int  maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));

  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
  assert(scalingListType < SCALING_LIST_NUM);
  Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrSize-2);

  const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));
  const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
    * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
    * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
    * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
    */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);
  if (useTransformSkip && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  // iAdd is different from the iAdd used in normal quantization
  const Int iAdd   = (compID == COMPONENT_Y ? 171 : 256) << (iQBits-9);

  for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )
  {
    const TCoeff iLevel   = piCoef[uiBlockPos];
    const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);

    if ( quantisedMagnitude != 0 )
    {
      return true;
    }
  } // for n
  return false;
}

Void TComTrQuant::xDeQuant(       TComTU        &rTu,
                            const TCoeff       * pSrc,
                                  TCoeff       * pDes,
                            const ComponentID    compID,
                            const QpParam       &cQP )
{
  assert(compID<MAX_NUM_COMPONENT);

        TComDataCU          *pcCU               = rTu.getCU();
  const UInt                 uiAbsPartIdx       = rTu.GetAbsPartIdxTU();
  const TComRectangle       &rect               = rTu.getRect(compID);
  const UInt                 uiWidth            = rect.width;
  const UInt                 uiHeight           = rect.height;
  const TCoeff        *const piQCoef            = pSrc;
        TCoeff        *const piCoef             = pDes;
  const UInt                 uiLog2TrSize       = rTu.GetEquivalentLog2TrSize(compID);
  const UInt                 numSamplesInBlock  = uiWidth*uiHeight;
  const Int                  maxLog2TrDynamicRange  = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff               transformMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff               transformMaximum   =  (1 << maxLog2TrDynamicRange) - 1;
  const Bool                 enableScalingLists = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));
  const Int                  scalingListType    = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
#if O0043_BEST_EFFORT_DECODING
  const Int                  channelBitDepth    = pcCU->getSlice()->getSPS()->getStreamBitDepth(toChannelType(compID));
#else
  const Int                  channelBitDepth    = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
#endif

  assert (scalingListType < SCALING_LIST_NUM);
  assert ( uiWidth <= m_uiMaxTrSize );

  // Represents scaling through forward transform
  const Bool bClipTransformShiftTo0 = (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0) && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const Int  originalTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);
  const Int  iTransformShift        = bClipTransformShiftTo0 ? std::max<Int>(0, originalTransformShift) : originalTransformShift;

  const Int QP_per = cQP.per;
  const Int QP_rem = cQP.rem;

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  if(enableScalingLists)
  {
    //from the dequantisation equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n]) + iAdd ) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n]) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
    const Int scale     =  g_invQuantScales[QP_rem];
    const Int scaleBits =     (IQUANT_SHIFT + 1)   ;

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
}


Void TComTrQuant::init(   UInt  uiMaxTrSize,
                          Bool  bUseRDOQ,
                          Bool  bUseRDOQTS,
                          Bool  useSelectiveRDOQ,
                          Bool  bEnc,
                          Bool  useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                        , Bool bUseAdaptQpSelect
#endif
                       )
{
  m_uiMaxTrSize  = uiMaxTrSize;
  m_bEnc         = bEnc;
  m_useRDOQ      = bUseRDOQ;
  m_useRDOQTS    = bUseRDOQTS;
  m_useSelectiveRDOQ = useSelectiveRDOQ;
#if ADAPTIVE_QP_SELECTION
  m_bUseAdaptQpSelect = bUseAdaptQpSelect;
#endif
  m_useTransformSkipFast = useTransformSkipFast;
}


Void TComTrQuant::transformNxN(       TComTU        & rTu,
                                const ComponentID     compID,
                                      Pel          *  pcResidual,
                                const UInt            uiStride,
                                      TCoeff       *  rpcCoeff,
#if ADAPTIVE_QP_SELECTION
                                      TCoeff       *  pcArlCoeff,
#endif
                                      TCoeff        & uiAbsSum,
                                const QpParam       & cQP
                              )
{
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const UInt uiOrgTrDepth   = rTu.GetTransformDepthRel();

  uiAbsSum=0;

  RDPCMMode rdpcmMode = RDPCM_OFF;
  rdpcmNxN( rTu, compID, pcResidual, uiStride, cQP, rpcCoeff, uiAbsSum, rdpcmMode );

  if (rdpcmMode == RDPCM_OFF)
  {
    uiAbsSum = 0;
    //transform and quantise
    if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
    {
      const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);
      const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

      for (UInt y = 0, coefficientIndex = 0; y<uiHeight; y++)
      {
        for (UInt x = 0; x<uiWidth; x++, coefficientIndex++)
        {
          const Pel currentSample = pcResidual[(y * uiStride) + x];

          rpcCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = currentSample;
          uiAbsSum += TCoeff(abs(currentSample));
        }
      }
    }
    else
    {
#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to transform\n";
      printBlock(pcResidual, uiWidth, uiHeight, uiStride);
#endif

      assert( (pcCU->getSlice()->getSPS()->getMaxTrSize() >= uiWidth) );

      if(pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0)
      {
        xTransformSkip( pcResidual, uiStride, m_plTempCoeff, rTu, compID );
      }
      else
      {
        const Int channelBitDepth=pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
        xT( channelBitDepth, rTu.useDST(compID), pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight, pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID)) );
      }

#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between transform and quantiser\n";
      printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

      xQuant( rTu, m_plTempCoeff, rpcCoeff,

#if ADAPTIVE_QP_SELECTION
              pcArlCoeff,
#endif
              uiAbsSum, compID, cQP );

#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of quantiser\n";
      printBlock(rpcCoeff, uiWidth, uiHeight, uiWidth);
#endif
    }
  }

    //set the CBF
  pcCU->setCbfPartRange((((uiAbsSum > 0) ? 1 : 0) << uiOrgTrDepth), compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(compID));
}


Void TComTrQuant::invTransformNxN(      TComTU        &rTu,
                                  const ComponentID    compID,
                                        Pel          *pcResidual,
                                  const UInt           uiStride,
                                        TCoeff       * pcCoeff,
                                  const QpParam       &cQP
                                        DEBUG_STRING_FN_DECLAREP(psDebug))
{
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth = rect.width;
  const UInt uiHeight = rect.height;

  if (uiWidth != uiHeight) //for intra, the TU will have been split above this level, so this condition won't be true, hence this only affects inter
  {
    //------------------------------------------------

    //recurse deeper

    TComTURecurse subTURecurse(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

    do
    {
      //------------------

      const UInt lineOffset = subTURecurse.GetSectionNumber() * subTURecurse.getRect(compID).height;

      Pel    *subTUResidual     = pcResidual + (lineOffset * uiStride);
      TCoeff *subTUCoefficients = pcCoeff     + (lineOffset * subTURecurse.getRect(compID).width);

      invTransformNxN(subTURecurse, compID, subTUResidual, uiStride, subTUCoefficients, cQP DEBUG_STRING_PASS_INTO(psDebug));

      //------------------

    } while (subTURecurse.nextSection(rTu));

    //------------------------------------------------

    return;
  }

#if DEBUG_STRING
  if (psDebug)
  {
    std::stringstream ss(stringstream::out);
    printBlockToStream(ss, (compID==0)?"###InvTran ip Ch0: " : ((compID==1)?"###InvTran ip Ch1: ":"###InvTran ip Ch2: "), pcCoeff, uiWidth, uiHeight, uiWidth);
    DEBUG_STRING_APPEND((*psDebug), ss.str())
  }
#endif

  if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);
    const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

    for (UInt y = 0, coefficientIndex = 0; y<uiHeight; y++)
    {
      for (UInt x = 0; x<uiWidth; x++, coefficientIndex++)
      {
        pcResidual[(y * uiStride) + x] = Pel(pcCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex]);
      }
    }
  }
  else
  {
#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to dequantiser\n";
    printBlock(pcCoeff, uiWidth, uiHeight, uiWidth);
#endif

    xDeQuant(rTu, pcCoeff, m_plTempCoeff, compID, cQP);

#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between dequantiser and inverse-transform\n";
    printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

#if DEBUG_STRING
    if (psDebug)
    {
      std::stringstream ss(stringstream::out);
      printBlockToStream(ss, "###InvTran deq: ", m_plTempCoeff, uiWidth, uiHeight, uiWidth);
      (*psDebug)+=ss.str();
    }
#endif

    if(pcCU->getTransformSkip(uiAbsPartIdx, compID))
    {
      xITransformSkip( m_plTempCoeff, pcResidual, uiStride, rTu, compID );

#if DEBUG_STRING
      if (psDebug)
      {
        std::stringstream ss(stringstream::out);
        printBlockToStream(ss, "###InvTran resi: ", pcResidual, uiWidth, uiHeight, uiStride);
        (*psDebug)+=ss.str();
        (*psDebug)+="(<- was a Transform-skipped block)\n";
      }
#endif
    }
    else
    {
#if O0043_BEST_EFFORT_DECODING
      const Int channelBitDepth = pcCU->getSlice()->getSPS()->getStreamBitDepth(toChannelType(compID));
#else
      const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
#endif
      xIT( channelBitDepth, rTu.useDST(compID), m_plTempCoeff, pcResidual, uiStride, uiWidth, uiHeight, pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID)) );

#if DEBUG_STRING
      if (psDebug)
      {
        std::stringstream ss(stringstream::out);
        printBlockToStream(ss, "###InvTran resi: ", pcResidual, uiWidth, uiHeight, uiStride);
        (*psDebug)+=ss.str();
        (*psDebug)+="(<- was a Transformed block)\n";
      }
#endif
    }

#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of inverse-transform\n";
    printBlock(pcResidual, uiWidth, uiHeight, uiStride);
    g_debugCounter++;
#endif
  }

  invRdpcmNxN( rTu, compID, pcResidual, uiStride );
}

Void TComTrQuant::invRecurTransformNxN( const ComponentID compID,
                                        TComYuv *pResidual,
                                        TComTU &rTu)
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }

  TComDataCU* pcCU = rTu.getCU();
  UInt absPartIdxTU = rTu.GetAbsPartIdxTU();
  UInt uiTrMode=rTu.GetTransformDepthRel();
  if( (pcCU->getCbf(absPartIdxTU, compID, uiTrMode) == 0) && (isLuma(compID) || !pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()) )
  {
    return;
  }

  if( uiTrMode == pcCU->getTransformIdx( absPartIdxTU ) )
  {
    const TComRectangle &tuRect      = rTu.getRect(compID);
    const Int            uiStride    = pResidual->getStride( compID );
          Pel           *rpcResidual = pResidual->getAddr( compID );
          UInt           uiAddr      = (tuRect.x0 + uiStride*tuRect.y0);
          Pel           *pResi       = rpcResidual + uiAddr;
          TCoeff        *pcCoeff     = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);

    const QpParam cQP(*pcCU, compID);

    if(pcCU->getCbf(absPartIdxTU, compID, uiTrMode) != 0)
    {
      DEBUG_STRING_NEW(sTemp)
#if DEBUG_STRING
      std::string *psDebug=((DebugOptionList::DebugString_InvTran.getInt()&(pcCU->isIntra(absPartIdxTU)?1:(pcCU->isInter(absPartIdxTU)?2:4)))!=0) ? &sTemp : 0;
#endif

      invTransformNxN( rTu, compID, pResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO(psDebug) );

#if DEBUG_STRING
      if (psDebug != 0)
      {
        std::cout << (*psDebug);
      }
#endif
    }

    if (isChroma(compID) && (pcCU->getCrossComponentPredictionAlpha(absPartIdxTU, compID) != 0))
    {
      const Pel *piResiLuma = pResidual->getAddr( COMPONENT_Y );
      const Int  strideLuma = pResidual->getStride( COMPONENT_Y );
      const Int  tuWidth    = rTu.getRect( compID ).width;
      const Int  tuHeight   = rTu.getRect( compID ).height;

      if(pcCU->getCbf(absPartIdxTU, COMPONENT_Y, uiTrMode) != 0)
      {
        pResi = rpcResidual + uiAddr;
        const Pel *pResiLuma = piResiLuma + uiAddr;

        crossComponentPrediction( rTu, compID, pResiLuma, pResi, pResi, tuWidth, tuHeight, strideLuma, uiStride, uiStride, true );
      }
    }
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      invRecurTransformNxN( compID, pResidual, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}

Void TComTrQuant::applyForwardRDPCM( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride, const QpParam& cQP, TCoeff* pcCoeff, TCoeff &uiAbsSum, const RDPCMMode mode )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  const Bool bLossless      = pcCU->getCUTransquantBypass( uiAbsPartIdx );
  const UInt uiWidth        = rTu.getRect(compID).width;
  const UInt uiHeight       = rTu.getRect(compID).height;
  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);
  const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

  UInt uiX = 0;
  UInt uiY = 0;

        UInt &majorAxis             = (mode == RDPCM_VER) ? uiX      : uiY;
        UInt &minorAxis             = (mode == RDPCM_VER) ? uiY      : uiX;
  const UInt  majorAxisLimit        = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const UInt  minorAxisLimit        = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const Bool bUseHalfRoundingPoint  = (mode != RDPCM_OFF);

  uiAbsSum = 0;

  for ( majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++ )
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator
    for ( minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++ )
    {
      const UInt sampleIndex      = (uiY * uiWidth) + uiX;
      const UInt coefficientIndex = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);
      const Pel  currentSample    = pcResidual[(uiY * uiStride) + uiX];
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;

      Pel reconstructedDelta;
      if ( bLossless )
      {
        pcCoeff[coefficientIndex] = encoderSideDelta;
        reconstructedDelta        = (Pel) encoderSideDelta;
      }
      else
      {
        transformSkipQuantOneSample(rTu, compID, encoderSideDelta, pcCoeff, coefficientIndex, cQP, bUseHalfRoundingPoint);
        invTrSkipDeQuantOneSample  (rTu, compID, pcCoeff[coefficientIndex], reconstructedDelta, cQP, coefficientIndex);
      }

      uiAbsSum += abs(pcCoeff[coefficientIndex]);

      if (mode != RDPCM_OFF)
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

Void TComTrQuant::rdpcmNxN   ( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride, const QpParam& cQP, TCoeff* pcCoeff, TCoeff &uiAbsSum, RDPCMMode& rdpcmMode )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  if (!pcCU->isRDPCMEnabled(uiAbsPartIdx) || ((pcCU->getTransformSkip(uiAbsPartIdx, compID) == 0) && !pcCU->getCUTransquantBypass(uiAbsPartIdx)))
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    const ChromaFormat chFmt = pcCU->getPic()->getPicYuvOrg()->getChromaFormat();
    const ChannelType chType = toChannelType(compID);
    const UInt uiChPredMode  = pcCU->getIntraDir( chType, uiAbsPartIdx );
    const TComSPS *sps=pcCU->getSlice()->getSPS();
    const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
    const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && isChroma(compID)) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
    const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && isChroma(compID)) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      applyForwardRDPCM( rTu, compID, pcResidual, uiStride, cQP, pcCoeff, uiAbsSum, rdpcmMode );
    }
    else
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode
  {
    const UInt uiWidth  = rTu.getRect(compID).width;
    const UInt uiHeight = rTu.getRect(compID).height;

    RDPCMMode bestMode   = NUMBER_OF_RDPCM_MODES;
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TU_SIZE * MAX_TU_SIZE];

    for (UInt modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM( rTu, compID, pcResidual, uiStride, cQP, pcCoeff, currAbsSum, mode );

      if (currAbsSum < bestAbsSum)
      {
        bestMode   = mode;
        bestAbsSum = currAbsSum;
        if (mode != RDPCM_OFF)
        {
          memcpy(bestCoefficients, pcCoeff, (uiWidth * uiHeight * sizeof(TCoeff)));
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum  = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantised if DPCM_OFF is returned, so there is no need to preserve it here
    {
      memcpy(pcCoeff, bestCoefficients, (uiWidth * uiHeight * sizeof(TCoeff)));
    }
  }

  pcCU->setExplicitRdpcmModePartRange(rdpcmMode, compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(compID));
}

Void TComTrQuant::invRdpcmNxN( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  if (pcCU->isRDPCMEnabled( uiAbsPartIdx ) && ((pcCU->getTransformSkip(uiAbsPartIdx, compID ) != 0) || pcCU->getCUTransquantBypass(uiAbsPartIdx)))
  {
    const UInt uiWidth  = rTu.getRect(compID).width;
    const UInt uiHeight = rTu.getRect(compID).height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if ( pcCU->isIntra( uiAbsPartIdx ) )
    {
      const ChromaFormat chFmt = pcCU->getPic()->getPicYuvRec()->getChromaFormat();
      const ChannelType chType = toChannelType(compID);
      const UInt uiChPredMode  = pcCU->getIntraDir( chType, uiAbsPartIdx );
      const TComSPS *sps=pcCU->getSlice()->getSPS();
      const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
      const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && isChroma(compID)) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
      const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && isChroma(compID)) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(pcCU->getExplicitRdpcmMode( compID, uiAbsPartIdx ));
    }

    const TCoeff pelMin=(TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax=(TCoeff) std::numeric_limits<Pel>::max();
    if (rdpcmMode == RDPCM_VER)
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        Pel *pcCurResidual = pcResidual+uiX;
        TCoeff accumulator = *pcCurResidual; // 32-bit accumulator
        pcCurResidual+=uiStride;
        for( UInt uiY = 1; uiY < uiHeight; uiY++, pcCurResidual+=uiStride )
        {
          accumulator += *(pcCurResidual);
          *pcCurResidual = (Pel)Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )
      {
        Pel *pcCurResidual = pcResidual+uiY*uiStride;
        TCoeff accumulator = *pcCurResidual;
        pcCurResidual++;
        for( UInt uiX = 1; uiX < uiWidth; uiX++, pcCurResidual++ )
        {
          accumulator += *(pcCurResidual);
          *pcCurResidual = (Pel)Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

/** Wrapper function between HM interface and core NxN forward transform (2D)
 *  \param channelBitDepth bit depth of channel
 *  \param useDST
 *  \param piBlkResi input data (residual)
 *  \param uiStride stride of input residual data
 *  \param psCoeff output data (transform coefficients)
 *  \param iWidth transform width
 *  \param iHeight transform height
 *  \param maxLog2TrDynamicRange
 */
Void TComTrQuant::xT( const Int channelBitDepth, Bool useDST, Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange )
{
#if MATRIX_MULT
  if( iWidth == iHeight)
  {
    xTr(channelBitDepth, piBlkResi, psCoeff, uiStride, (UInt)iWidth, useDST, maxLog2TrDynamicRange);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      block[(y * iWidth) + x] = piBlkResi[(y * uiStride) + x];
    }
  }

  xTrMxN( channelBitDepth, block, coeff, iWidth, iHeight, useDST, maxLog2TrDynamicRange );

  memcpy(psCoeff, coeff, (iWidth * iHeight * sizeof(TCoeff)));
}

/** Wrapper function between HM interface and core NxN inverse transform (2D)
 *  \param channelBitDepth bit depth of channel
 *  \param useDST
 *  \param plCoef input data (transform coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iWidth transform width
 *  \param iHeight transform height
 *  \param maxLog2TrDynamicRange
 */
Void TComTrQuant::xIT( const Int channelBitDepth, Bool useDST, TCoeff* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange )
{
#if MATRIX_MULT
  if( iWidth == iHeight )
  {
    xITr(channelBitDepth, plCoef, pResidual, uiStride, (UInt)iWidth, useDST, maxLog2TrDynamicRange);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  memcpy(coeff, plCoef, (iWidth * iHeight * sizeof(TCoeff)));

  xITrMxN( channelBitDepth, coeff, block, iWidth, iHeight, useDST, maxLog2TrDynamicRange );

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      pResidual[(y * uiStride) + x] = Pel(block[(y * iWidth) + x]);
    }
  }
}

/** Wrapper function between HM interface and core 4x4 transform skipping
 *  \param piBlkResi input data (residual)
 *  \param uiStride stride of input residual data
 *  \param psCoeff output data (transform coefficients)
 *  \param rTu reference to transform data
 *  \param component colour component
 */
Void TComTrQuant::xTransformSkip( Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, TComTU &rTu, const ComponentID component )
{
  const TComRectangle &rect = rTu.getRect(component);
  const Int width           = rect.width;
  const Int height          = rect.height;
  const Int maxLog2TrDynamicRange = rTu.getCU()->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(component));
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(toChannelType(component));

  Int iTransformShift = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(component), maxLog2TrDynamicRange);
  if (rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(component);
  const UInt uiSizeMinus1   = (width * height) - 1;

  if (iTransformShift >= 0)
  {
    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        psCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = TCoeff(piBlkResi[(y * uiStride) + x]) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << (iTransformShift - 1);

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        psCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = (TCoeff(piBlkResi[(y * uiStride) + x]) + offset) >> iTransformShift;
      }
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping
 *  \param plCoef input data (coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param rTu reference to transform data
 *  \param component colour component ID
 */
Void TComTrQuant::xITransformSkip( TCoeff* plCoef, Pel* pResidual, UInt uiStride, TComTU &rTu, const ComponentID component )
{
  const TComRectangle &rect = rTu.getRect(component);
  const Int width           = rect.width;
  const Int height          = rect.height;
  const Int maxLog2TrDynamicRange = rTu.getCU()->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(component));
#if O0043_BEST_EFFORT_DECODING
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(toChannelType(component));
#else
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(toChannelType(component));
#endif

  Int iTransformShift = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(component), maxLog2TrDynamicRange);
  if (rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(component);
  const UInt uiSizeMinus1   = (width * height) - 1;

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift==0 ? 0 : (1 << (iTransformShift - 1));

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        pResidual[(y * uiStride) + x] =  Pel((plCoef[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] + offset) >> iTransformShift);
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
