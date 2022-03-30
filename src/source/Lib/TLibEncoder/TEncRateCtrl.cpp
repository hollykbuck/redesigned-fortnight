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

/** \file     TEncRateCtrl.cpp
    \brief    Rate control manager class
*/
#include "TEncRateCtrl.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComChromaFormat.h"

#include <cmath>

using namespace std;

//sequence level
TEncRCSeq::TEncRCSeq()
{
  m_totalFrames         = 0;
  m_targetRate          = 0;
  m_frameRate           = 0;
  m_targetBits          = 0;
  m_GOPSize             = 0;
#if JVET_Y0105_SW_AND_QDF
  m_intraPeriod         = 0;
#endif
  m_picWidth            = 0;
  m_picHeight           = 0;
  m_LCUWidth            = 0;
  m_LCUHeight           = 0;
  m_numberOfLevel       = 0;
  m_numberOfLCU         = 0;
  m_averageBits         = 0;
  m_bitsRatio           = NULL;
  m_GOPID2Level         = NULL;
  m_picPara             = NULL;
  m_LCUPara             = NULL;
  m_numberOfPixel       = 0;
  m_framesLeft          = 0;
  m_bitsLeft            = 0;
  m_useLCUSeparateModel = false;
  m_adaptiveBit         = 0;
  m_lastLambda          = 0.0;
}

TEncRCSeq::~TEncRCSeq()
{
  destroy();
}

#if JVET_Y0105_SW_AND_QDF
Void TEncRCSeq::create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int intraPeriod, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit )
#else
Void TEncRCSeq::create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit )
#endif
{
  destroy();
  m_totalFrames         = totalFrames;
  m_targetRate          = targetBitrate;
  m_frameRate           = frameRate;
  m_GOPSize             = GOPSize;
#if JVET_Y0105_SW_AND_QDF
  m_intraPeriod         = intraPeriod;
#endif
  m_picWidth            = picWidth;
  m_picHeight           = picHeight;
  m_LCUWidth            = LCUWidth;
  m_LCUHeight           = LCUHeight;
  m_numberOfLevel       = numberOfLevel;
  m_useLCUSeparateModel = useLCUSeparateModel;

  m_numberOfPixel   = m_picWidth * m_picHeight;
  m_targetBits      = (Int64)m_totalFrames * (Int64)m_targetRate / (Int64)m_frameRate;
  m_seqTargetBpp = (Double)m_targetRate / (Double)m_frameRate / (Double)m_numberOfPixel;
  if ( m_seqTargetBpp < 0.03 )
  {
    m_alphaUpdate = 0.01;
    m_betaUpdate  = 0.005;
  }
  else if ( m_seqTargetBpp < 0.08 )
  {
    m_alphaUpdate = 0.05;
    m_betaUpdate  = 0.025;
  }
  else if ( m_seqTargetBpp < 0.2 )
  {
    m_alphaUpdate = 0.1;
    m_betaUpdate  = 0.05;
  }
  else if ( m_seqTargetBpp < 0.5 )
  {
    m_alphaUpdate = 0.2;
    m_betaUpdate  = 0.1;
  }
  else
  {
    m_alphaUpdate = 0.4;
    m_betaUpdate  = 0.2;
  }

  m_averageBits     = (Int)(m_targetBits / totalFrames);
  Int picWidthInBU  = ( m_picWidth  % m_LCUWidth  ) == 0 ? m_picWidth  / m_LCUWidth  : m_picWidth  / m_LCUWidth  + 1;
  Int picHeightInBU = ( m_picHeight % m_LCUHeight ) == 0 ? m_picHeight / m_LCUHeight : m_picHeight / m_LCUHeight + 1;
  m_numberOfLCU     = picWidthInBU * picHeightInBU;

  m_bitsRatio   = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_bitsRatio[i] = 1;
  }

  m_GOPID2Level = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = 1;
  }

  m_picPara = new TRCParameter[m_numberOfLevel];
  for ( Int i=0; i<m_numberOfLevel; i++ )
  {
    m_picPara[i].m_alpha = 0.0;
    m_picPara[i].m_beta  = 0.0;
#if JVET_K0390_RATE_CTRL
    m_picPara[i].m_validPix = -1;
#endif
#if JVET_M0600_RATE_CTRL
    m_picPara[i].m_skipRatio = 0.0;
#endif
  }

  if ( m_useLCUSeparateModel )
  {
    m_LCUPara = new TRCParameter*[m_numberOfLevel];
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_LCUPara[i] = new TRCParameter[m_numberOfLCU];
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = 0.0;
        m_LCUPara[i][j].m_beta  = 0.0;
#if JVET_K0390_RATE_CTRL
        m_LCUPara[i][j].m_validPix = -1;
#endif
#if JVET_M0600_RATE_CTRL
        m_LCUPara[i][j].m_skipRatio = 0.0;
#endif
      }
    }
  }

  m_framesLeft = m_totalFrames;
  m_bitsLeft   = m_targetBits;
  m_adaptiveBit = adaptiveBit;
  m_lastLambda = 0.0;
}

Void TEncRCSeq::destroy()
{
  if (m_bitsRatio != NULL)
  {
    delete[] m_bitsRatio;
    m_bitsRatio = NULL;
  }

  if ( m_GOPID2Level != NULL )
  {
    delete[] m_GOPID2Level;
    m_GOPID2Level = NULL;
  }

  if ( m_picPara != NULL )
  {
    delete[] m_picPara;
    m_picPara = NULL;
  }

  if ( m_LCUPara != NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      delete[] m_LCUPara[i];
    }
    delete[] m_LCUPara;
    m_LCUPara = NULL;
  }
}

Void TEncRCSeq::initBitsRatio( Int bitsRatio[])
{
  for (Int i=0; i<m_GOPSize; i++)
  {
    m_bitsRatio[i] = bitsRatio[i];
  }
}

Void TEncRCSeq::initGOPID2Level( Int GOPID2Level[] )
{
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = GOPID2Level[i];
  }
}

Void TEncRCSeq::initPicPara( TRCParameter* picPara )
{
  assert( m_picPara != NULL );

  if ( picPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      if (i>0)
      {
        m_picPara[i].m_alpha = 3.2003;
        m_picPara[i].m_beta  = -1.367;
      }
      else
      {
        m_picPara[i].m_alpha = ALPHA;
        m_picPara[i].m_beta  = BETA2;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_picPara[i] = picPara[i];
    }
  }
}

Void TEncRCSeq::initLCUPara( TRCParameter** LCUPara )
{
  if ( m_LCUPara == NULL )
  {
    return;
  }
  if ( LCUPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = m_picPara[i].m_alpha;
        m_LCUPara[i][j].m_beta  = m_picPara[i].m_beta;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j] = LCUPara[i][j];
      }
    }
  }
}

Void TEncRCSeq::updateAfterPic ( Int bits )
{
  m_bitsLeft -= bits;
  m_framesLeft--;
}

Void TEncRCSeq::setAllBitRatio( Double basicLambda, Double* equaCoeffA, Double* equaCoeffB )
{
  Int* bitsRatio = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
#if JVET_K0390_RATE_CTRL
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow(basicLambda, equaCoeffB[i]) * (Double)getPicPara(getGOPID2Level(i)).m_validPix);
#else
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow( basicLambda, equaCoeffB[i] ) * m_numberOfPixel );
#endif
  }
  initBitsRatio( bitsRatio );
  delete[] bitsRatio;
}

//GOP level
TEncRCGOP::TEncRCGOP()
{
  m_encRCSeq  = NULL;
  m_picTargetBitInGOP = NULL;
  m_numPic     = 0;
  m_targetBits = 0;
  m_picLeft    = 0;
  m_bitsLeft   = 0;
}

TEncRCGOP::~TEncRCGOP()
{
  destroy();
}

#if JVET_Y0105_SW_AND_QDF
Void TEncRCGOP::create( TEncRCSeq* encRCSeq, Int numPic, Bool useAdaptiveBitsRatio )
#else
Void TEncRCGOP::create( TEncRCSeq* encRCSeq, Int numPic )
#endif
{
  destroy();
  Int targetBits = xEstGOPTargetBits( encRCSeq, numPic );



#if JVET_Y0105_SW_AND_QDF
  if ( useAdaptiveBitsRatio )
#else
  if ( encRCSeq->getAdaptiveBits() > 0 && encRCSeq->getLastLambda() > 0.1 )
#endif
  {
    Double targetBpp = (Double)targetBits / encRCSeq->getNumPixel();
    Double basicLambda = 0.0;
    Double* lambdaRatio = new Double[encRCSeq->getGOPSize()];
    Double* equaCoeffA = new Double[encRCSeq->getGOPSize()];
    Double* equaCoeffB = new Double[encRCSeq->getGOPSize()];

    if ( encRCSeq->getAdaptiveBits() == 1 )   // for GOP size =4, low delay case
    {
      if ( encRCSeq->getLastLambda() < 120.0 )
      {
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.5793;
        lambdaRatio[0] = 1.3 * lambdaRatio[1];
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 1.0;
      }
      else
      {
        lambdaRatio[0] = 5.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 1.0;
      }
    }
    else if ( encRCSeq->getAdaptiveBits() == 2 )  // for GOP size = 8, random access case
    {
      if ( encRCSeq->getLastLambda() < 90.0 )
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.7963;
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 3.25 * lambdaRatio[1];
        lambdaRatio[4] = 3.25 * lambdaRatio[1];
        lambdaRatio[5] = 1.3  * lambdaRatio[1];
        lambdaRatio[6] = 3.25 * lambdaRatio[1];
        lambdaRatio[7] = 3.25 * lambdaRatio[1];
      }
      else
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 12.3;
        lambdaRatio[4] = 12.3;
        lambdaRatio[5] = 5.0;
        lambdaRatio[6] = 12.3;
        lambdaRatio[7] = 12.3;
      }
    }
#if JVET_K0390_RATE_CTRL
    else if (encRCSeq->getAdaptiveBits() == 3)  // for GOP size = 16, random access case
    {
      Double hierarQp = 4.2005 * log(encRCSeq->getLastLambda()) + 13.7122;  //  the qp of POC16
      Double qpLev2 = (hierarQp + 0.0) + 0.2016    * (hierarQp + 0.0) - 4.8848;
      Double qpLev3 = (hierarQp + 3.0) + 0.22286 * (hierarQp + 3.0) - 5.7476;
      Double qpLev4 = (hierarQp + 4.0) + 0.2333    * (hierarQp + 4.0) - 5.9;
      Double qpLev5 = (hierarQp + 5.0) + 0.3            * (hierarQp + 5.0) - 7.1444;

      Double lambdaLev1 = exp((hierarQp - 13.7122) / 4.2005);
      Double lambdaLev2 = exp((qpLev2 - 13.7122) / 4.2005);
      Double lambdaLev3 = exp((qpLev3 - 13.7122) / 4.2005);
      Double lambdaLev4 = exp((qpLev4 - 13.7122) / 4.2005);
      Double lambdaLev5 = exp((qpLev5 - 13.7122) / 4.2005);

      lambdaRatio[0] = 1.0;
      lambdaRatio[1] = lambdaLev2 / lambdaLev1;
      lambdaRatio[2] = lambdaLev3 / lambdaLev1;
      lambdaRatio[3] = lambdaLev4 / lambdaLev1;
      lambdaRatio[4] = lambdaLev5 / lambdaLev1;
      lambdaRatio[5] = lambdaLev5 / lambdaLev1;
      lambdaRatio[6] = lambdaLev4 / lambdaLev1;
      lambdaRatio[7] = lambdaLev5 / lambdaLev1;
      lambdaRatio[8] = lambdaLev5 / lambdaLev1;
      lambdaRatio[9] = lambdaLev3 / lambdaLev1;
      lambdaRatio[10] = lambdaLev4 / lambdaLev1;
      lambdaRatio[11] = lambdaLev5 / lambdaLev1;
      lambdaRatio[12] = lambdaLev5 / lambdaLev1;
      lambdaRatio[13] = lambdaLev4 / lambdaLev1;
      lambdaRatio[14] = lambdaLev5 / lambdaLev1;
      lambdaRatio[15] = lambdaLev5 / lambdaLev1;
#if JVET_M0600_RATE_CTRL
      const Double qdfParaLev2A = 0.5847;
      const Double qdfParaLev2B = -0.0782;
      const Double qdfParaLev3A = 0.5468;
      const Double qdfParaLev3B = -0.1364;
      const Double qdfParaLev4A = 0.6539;
      const Double qdfParaLev4B = -0.203;
      const Double qdfParaLev5A = 0.8623;
      const Double qdfParaLev5B = -0.4676;
      Double qdfLev1Lev2 = Clip3(0.12, 0.9, qdfParaLev2A * encRCSeq->getPicPara(2).m_skipRatio + qdfParaLev2B);
      Double qdfLev1Lev3 = Clip3(0.13, 0.9, qdfParaLev3A * encRCSeq->getPicPara(3).m_skipRatio + qdfParaLev3B);
      Double qdfLev1Lev4 = Clip3(0.15, 0.9, qdfParaLev4A * encRCSeq->getPicPara(4).m_skipRatio + qdfParaLev4B);
      Double qdfLev1Lev5 = Clip3(0.20, 0.9, qdfParaLev5A * encRCSeq->getPicPara(5).m_skipRatio + qdfParaLev5B);
      Double qdfLev2Lev3 = Clip3(0.09, 0.9, qdfLev1Lev3 * (1 - qdfLev1Lev2));
      Double qdfLev2Lev4 = Clip3(0.12, 0.9, qdfLev1Lev4 * (1 - qdfLev1Lev2));
      Double qdfLev2Lev5 = Clip3(0.14, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev2));
      Double qdfLev3Lev4 = Clip3(0.06, 0.9, qdfLev1Lev4 * (1 - qdfLev1Lev3));
      Double qdfLev3Lev5 = Clip3(0.09, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev3));
      Double qdfLev4Lev5 = Clip3(0.10, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev4));

      lambdaLev1 = 1 / (1 + 2 * (qdfLev1Lev2 + 2 * qdfLev1Lev3 + 4 * qdfLev1Lev4 + 8 * qdfLev1Lev5));
      lambdaLev2 = 1 / (1 + (3 * qdfLev2Lev3 + 5 * qdfLev2Lev4 + 8 * qdfLev2Lev5));
      lambdaLev3 = 1 / (1 + 2 * qdfLev3Lev4 + 4 * qdfLev3Lev5);
      lambdaLev4 = 1 / (1 + 2 * qdfLev4Lev5);
      lambdaLev5 = 1 / (1.0);

      lambdaRatio[0] = 1.0;
      lambdaRatio[1] = lambdaLev2 / lambdaLev1;
      lambdaRatio[2] = lambdaLev3 / lambdaLev1;
      lambdaRatio[3] = lambdaLev4 / lambdaLev1;
      lambdaRatio[4] = lambdaLev5 / lambdaLev1;
      lambdaRatio[5] = lambdaLev5 / lambdaLev1;
      lambdaRatio[6] = lambdaLev4 / lambdaLev1;
      lambdaRatio[7] = lambdaLev5 / lambdaLev1;
      lambdaRatio[8] = lambdaLev5 / lambdaLev1;
      lambdaRatio[9] = lambdaLev3 / lambdaLev1;
      lambdaRatio[10] = lambdaLev4 / lambdaLev1;
      lambdaRatio[11] = lambdaLev5 / lambdaLev1;
      lambdaRatio[12] = lambdaLev5 / lambdaLev1;
      lambdaRatio[13] = lambdaLev4 / lambdaLev1;
      lambdaRatio[14] = lambdaLev5 / lambdaLev1;
      lambdaRatio[15] = lambdaLev5 / lambdaLev1;
#endif
    }
#endif
    xCalEquaCoeff( encRCSeq, lambdaRatio, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#if JVET_K0390_RATE_CTRL
    basicLambda = xSolveEqua(encRCSeq, targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize());
#else
    basicLambda = xSolveEqua( targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#endif
    encRCSeq->setAllBitRatio( basicLambda, equaCoeffA, equaCoeffB );

    delete []lambdaRatio;
    delete []equaCoeffA;
    delete []equaCoeffB;
  }

  m_picTargetBitInGOP = new Int[numPic];
  Int i;
  Int totalPicRatio = 0;
  Int currPicRatio = 0;
  for ( i=0; i<numPic; i++ )
  {
    totalPicRatio += encRCSeq->getBitRatio( i );
  }
  for ( i=0; i<numPic; i++ )
  {
    currPicRatio = encRCSeq->getBitRatio( i );
    m_picTargetBitInGOP[i] = (Int)( ((Double)targetBits) * currPicRatio / totalPicRatio );
  }

  m_encRCSeq    = encRCSeq;
  m_numPic       = numPic;
  m_targetBits   = targetBits;
  m_picLeft      = m_numPic;
  m_bitsLeft     = m_targetBits;
}

Void TEncRCGOP::xCalEquaCoeff( TEncRCSeq* encRCSeq, Double* lambdaRatio, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
{
  for ( Int i=0; i<GOPSize; i++ )
  {
    Int frameLevel = encRCSeq->getGOPID2Level(i);
    Double alpha   = encRCSeq->getPicPara(frameLevel).m_alpha;
    Double beta    = encRCSeq->getPicPara(frameLevel).m_beta;
    equaCoeffA[i] = pow( 1.0/alpha, 1.0/beta ) * pow( lambdaRatio[i], 1.0/beta );
    equaCoeffB[i] = 1.0/beta;
  }
}

#if JVET_K0390_RATE_CTRL
Double TEncRCGOP::xSolveEqua(TEncRCSeq* encRCSeq, Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize)
#else
Double TEncRCGOP::xSolveEqua( Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
#endif
{
  Double solution = 100.0;
  Double minNumber = 0.1;
  Double maxNumber = 10000.0;
  for ( Int i=0; i<g_RCIterationNum; i++ )
  {
    Double fx = 0.0;
    for ( Int j=0; j<GOPSize; j++ )
    {
#if JVET_K0390_RATE_CTRL
      Double tmpBpp = equaCoeffA[j] * pow(solution, equaCoeffB[j]);
      Double actualBpp = tmpBpp * (Double)encRCSeq->getPicPara(encRCSeq->getGOPID2Level(j)).m_validPix / (Double)encRCSeq->getNumPixel();
      fx += actualBpp;
#else
      fx += equaCoeffA[j] * pow( solution, equaCoeffB[j] );
#endif
    }

    if ( fabs( fx - targetBpp ) < 0.000001 )
    {
      break;
    }

    if ( fx > targetBpp )
    {
      minNumber = solution;
      solution = ( solution + maxNumber ) / 2.0;
    }
    else
    {
      maxNumber = solution;
      solution = ( solution + minNumber ) / 2.0;
    }
  }

  solution = Clip3( 0.1, 10000.0, solution );
  return solution;
}

Void TEncRCGOP::destroy()
{
  m_encRCSeq = NULL;
  if ( m_picTargetBitInGOP != NULL )
  {
    delete[] m_picTargetBitInGOP;
    m_picTargetBitInGOP = NULL;
  }
}

Void TEncRCGOP::updateAfterPicture( Int bitsCost )
{
  m_bitsLeft -= bitsCost;
  m_picLeft--;
}

Int TEncRCGOP::xEstGOPTargetBits( TEncRCSeq* encRCSeq, Int GOPSize )
{
#if JVET_Y0105_SW_AND_QDF
  Int realInfluencePicture = min( g_RCSmoothWindowSizeAlpha * GOPSize / max(encRCSeq->getIntraPeriod(), 32) + g_RCSmoothWindowSizeBeta, encRCSeq->getFramesLeft() );
#else
  Int realInfluencePicture = min( g_RCSmoothWindowSize, encRCSeq->getFramesLeft() );
#endif
  Int averageTargetBitsPerPic = (Int)( encRCSeq->getTargetBits() / encRCSeq->getTotalFrames() );
  Int currentTargetBitsPerPic = (Int)( ( encRCSeq->getBitsLeft() - averageTargetBitsPerPic * (encRCSeq->getFramesLeft() - realInfluencePicture) ) / realInfluencePicture );
  Int targetBits = currentTargetBitsPerPic * GOPSize;

  if ( targetBits < 200 )
  {
    targetBits = 200;   // at least allocate 200 bits for one GOP
  }

  return targetBits;
}

//picture level
TEncRCPic::TEncRCPic()
{
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;

  m_frameLevel    = 0;
