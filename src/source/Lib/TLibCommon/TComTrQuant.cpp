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
