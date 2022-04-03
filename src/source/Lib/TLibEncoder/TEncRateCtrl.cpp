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
  m_numberOfPixel = 0;
  m_numberOfLCU   = 0;
  m_targetBits    = 0;
  m_estHeaderBits = 0;
  m_estPicQP      = 0;
  m_estPicLambda  = 0.0;

  m_LCULeft       = 0;
  m_bitsLeft      = 0;
  m_pixelsLeft    = 0;

  m_LCUs         = NULL;
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
#if JVET_K0390_RATE_CTRL
  m_picMSE = 0.0;
  m_validPixelsInPic = 0;
#endif
}

TEncRCPic::~TEncRCPic()
{
  destroy();
}

Int TEncRCPic::xEstPicTargetBits( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP )
{
  Int targetBits        = 0;
  Int GOPbitsLeft       = encRCGOP->getBitsLeft();

  Int i;
  Int currPicPosition = encRCGOP->getNumPic()-encRCGOP->getPicLeft();
  Int currPicRatio    = encRCSeq->getBitRatio( currPicPosition );
  Int totalPicRatio   = 0;
  for ( i=currPicPosition; i<encRCGOP->getNumPic(); i++ )
  {
    totalPicRatio += encRCSeq->getBitRatio( i );
  }

  targetBits  = Int( ((Double)GOPbitsLeft) * currPicRatio / totalPicRatio );

  if ( targetBits < 100 )
  {
    targetBits = 100;   // at least allocate 100 bits for one picture
  }

  if ( m_encRCSeq->getFramesLeft() > 16 )
  {
    targetBits = Int( g_RCWeightPicRargetBitInBuffer * targetBits + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP( currPicPosition ) );
  }

  return targetBits;
}

Int TEncRCPic::xEstPicHeaderBits( list<TEncRCPic*>& listPreviousPictures, Int frameLevel )
{
  Int numPreviousPics   = 0;
  Int totalPreviousBits = 0;

  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == frameLevel )
    {
      totalPreviousBits += (*it)->getPicActualHeaderBits();
      numPreviousPics++;
    }
  }

  Int estHeaderBits = 0;
  if ( numPreviousPics > 0 )
  {
    estHeaderBits = totalPreviousBits / numPreviousPics;
  }

  return estHeaderBits;
}

Int TEncRCPic::xEstPicLowerBound(TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP)
{
  Int lowerBound = 0;
  Int GOPbitsLeft = encRCGOP->getBitsLeft();

  const Int nextPicPosition = (encRCGOP->getNumPic() - encRCGOP->getPicLeft() + 1) % encRCGOP->getNumPic();
  const Int nextPicRatio = encRCSeq->getBitRatio(nextPicPosition);

  Int totalPicRatio = 0;
  for (Int i = nextPicPosition; i < encRCGOP->getNumPic(); i++)
  {
    totalPicRatio += encRCSeq->getBitRatio(i);
  }

  if (nextPicPosition == 0)
  {
    GOPbitsLeft = encRCGOP->getTargetBits();
  }
  else
  {
    GOPbitsLeft -= m_targetBits;
  }

  lowerBound = Int(((Double)GOPbitsLeft) * nextPicRatio / totalPicRatio);

  if (lowerBound < 100)
  {
    lowerBound = 100;   // at least allocate 100 bits for one picture
  }

  if (m_encRCSeq->getFramesLeft() > 16)
  {
    lowerBound = Int(g_RCWeightPicRargetBitInBuffer * lowerBound + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP(nextPicPosition));
  }

  return lowerBound;
}

Void TEncRCPic::addToPictureLsit( list<TEncRCPic*>& listPreviousPictures )
{
  if ( listPreviousPictures.size() > g_RCMaxPicListSize )
  {
    TEncRCPic* p = listPreviousPictures.front();
    listPreviousPictures.pop_front();
    p->destroy();
    delete p;
  }

  listPreviousPictures.push_back( this );
}

Void TEncRCPic::create( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP, Int frameLevel, list<TEncRCPic*>& listPreviousPictures )
{
  destroy();
  m_encRCSeq = encRCSeq;
  m_encRCGOP = encRCGOP;

  Int targetBits    = xEstPicTargetBits( encRCSeq, encRCGOP );
  Int estHeaderBits = xEstPicHeaderBits( listPreviousPictures, frameLevel );

  if ( targetBits < estHeaderBits + 100 )
  {
    targetBits = estHeaderBits + 100;   // at least allocate 100 bits for picture data
  }

  m_frameLevel       = frameLevel;
  m_numberOfPixel    = encRCSeq->getNumPixel();
  m_numberOfLCU      = encRCSeq->getNumberOfLCU();
  m_estPicLambda     = 100.0;
  m_targetBits       = targetBits;
  m_estHeaderBits    = estHeaderBits;
  m_bitsLeft         = m_targetBits;
  Int picWidth       = encRCSeq->getPicWidth();
  Int picHeight      = encRCSeq->getPicHeight();
  Int LCUWidth       = encRCSeq->getLCUWidth();
  Int LCUHeight      = encRCSeq->getLCUHeight();
  Int picWidthInLCU  = ( picWidth  % LCUWidth  ) == 0 ? picWidth  / LCUWidth  : picWidth  / LCUWidth  + 1;
  Int picHeightInLCU = ( picHeight % LCUHeight ) == 0 ? picHeight / LCUHeight : picHeight / LCUHeight + 1;
  m_lowerBound       = xEstPicLowerBound( encRCSeq, encRCGOP );

  m_LCULeft         = m_numberOfLCU;
  m_bitsLeft       -= m_estHeaderBits;
  m_pixelsLeft      = m_numberOfPixel;

  m_LCUs           = new TRCLCU[m_numberOfLCU];
  Int i, j;
  Int LCUIdx;
  for ( i=0; i<picWidthInLCU; i++ )
  {
    for ( j=0; j<picHeightInLCU; j++ )
    {
      LCUIdx = j*picWidthInLCU + i;
      m_LCUs[LCUIdx].m_actualBits = 0;
#if JVET_K0390_RATE_CTRL
      m_LCUs[LCUIdx].m_actualSSE = 0.0;
      m_LCUs[LCUIdx].m_actualMSE = 0.0;
#endif
      m_LCUs[LCUIdx].m_QP         = 0;
      m_LCUs[LCUIdx].m_lambda     = 0.0;
      m_LCUs[LCUIdx].m_targetBits = 0;
      m_LCUs[LCUIdx].m_bitWeight  = 1.0;
      Int currWidth  = ( (i == picWidthInLCU -1) ? picWidth  - LCUWidth *(picWidthInLCU -1) : LCUWidth  );
      Int currHeight = ( (j == picHeightInLCU-1) ? picHeight - LCUHeight*(picHeightInLCU-1) : LCUHeight );
      m_LCUs[LCUIdx].m_numberOfPixel = currWidth * currHeight;
    }
  }
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
}

Void TEncRCPic::destroy()
{
  if( m_LCUs != NULL )
  {
    delete[] m_LCUs;
    m_LCUs = NULL;
  }
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;
}


Double TEncRCPic::estimatePicLambda( list<TEncRCPic*>& listPreviousPictures, SliceType eSliceType)
{
  Double alpha         = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double beta          = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
  Double bpp       = (Double)m_targetBits/(Double)m_numberOfPixel;
  
#if JVET_K0390_RATE_CTRL
  Int lastPicValPix = 0;
  if (listPreviousPictures.size() > 0)
  {
    lastPicValPix = m_encRCSeq->getPicPara(m_frameLevel).m_validPix;
  }
  if (lastPicValPix > 0)
  {
    bpp = (Double)m_targetBits / (Double)lastPicValPix;
  }
#endif
  
  Double estLambda;
  if (eSliceType == I_SLICE)
  {
    estLambda = calculateLambdaIntra(alpha, beta, pow(m_totalCostIntra/(Double)m_numberOfPixel, BETA1), bpp);
  }
  else
  {
    estLambda = alpha * pow( bpp, beta );
  }

  Double lastLevelLambda = -1.0;
  Double lastPicLambda   = -1.0;
  Double lastValidLambda = -1.0;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == m_frameLevel )
    {
      lastLevelLambda = (*it)->getPicActualLambda();
    }
    lastPicLambda     = (*it)->getPicActualLambda();

    if ( lastPicLambda > 0.0 )
    {
      lastValidLambda = lastPicLambda;
    }
  }

  if ( lastLevelLambda > 0.0 )
  {
    lastLevelLambda = Clip3( 0.1, 10000.0, lastLevelLambda );
    estLambda = Clip3( lastLevelLambda * pow( 2.0, -3.0/3.0 ), lastLevelLambda * pow( 2.0, 3.0/3.0 ), estLambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( 0.1, 2000.0, lastPicLambda );
    estLambda = Clip3( lastPicLambda * pow( 2.0, -10.0/3.0 ), lastPicLambda * pow( 2.0, 10.0/3.0 ), estLambda );
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( 0.1, 2000.0, lastValidLambda );
    estLambda = Clip3( lastValidLambda * pow(2.0, -10.0/3.0), lastValidLambda * pow(2.0, 10.0/3.0), estLambda );
  }
  else
  {
    estLambda = Clip3( 0.1, 10000.0, estLambda );
  }

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1;
  }
#if JVET_K0390_RATE_CTRL
  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = Double(int64_t(estLambda * (Double)LAMBDA_PREC + 0.5)) / (Double)LAMBDA_PREC;
#endif
  m_estPicLambda = estLambda;

  Double totalWeight = 0.0;
  // initial BU bit allocation weight
  for ( Int i=0; i<m_numberOfLCU; i++ )
  {
    Double alphaLCU, betaLCU;
    if ( m_encRCSeq->getUseLCUSeparateModel() )
    {
      alphaLCU = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_alpha;
      betaLCU  = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_beta;
    }
    else
    {
      alphaLCU = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
      betaLCU  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
    }

    m_LCUs[i].m_bitWeight =  m_LCUs[i].m_numberOfPixel * pow( estLambda/alphaLCU, 1.0/betaLCU );

    if ( m_LCUs[i].m_bitWeight < 0.01 )
    {
      m_LCUs[i].m_bitWeight = 0.01;
    }
    totalWeight += m_LCUs[i].m_bitWeight;
  }
  for ( Int i=0; i<m_numberOfLCU; i++ )
  {
    Double BUTargetBits = m_targetBits * m_LCUs[i].m_bitWeight / totalWeight;
    m_LCUs[i].m_bitWeight = BUTargetBits;
  }

  return estLambda;
}

Int TEncRCPic::estimatePicQP( Double lambda, list<TEncRCPic*>& listPreviousPictures )
{
  Int QP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 );

  Int lastLevelQP = g_RCInvalidQPValue;
  Int lastPicQP   = g_RCInvalidQPValue;
  Int lastValidQP = g_RCInvalidQPValue;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == m_frameLevel )
    {
      lastLevelQP = (*it)->getPicActualQP();
    }
    lastPicQP = (*it)->getPicActualQP();
    if ( lastPicQP > g_RCInvalidQPValue )
    {
      lastValidQP = lastPicQP;
    }
  }

  if ( lastLevelQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastLevelQP - 3, lastLevelQP + 3, QP );
  }

  if( lastPicQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP );
  }
  else if( lastValidQP > g_RCInvalidQPValue )
  {
    QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP );
  }

  return QP;
}

Double TEncRCPic::getLCUTargetBpp(SliceType eSliceType)
{
  Int   LCUIdx    = getLCUCoded();
  Double bpp      = -1.0;
  Int avgBits     = 0;

  if (eSliceType == I_SLICE)
  {
    Int noOfLCUsLeft = m_numberOfLCU - LCUIdx + 1;
    Int bitrateWindow = min(4,noOfLCUsLeft);
    Double MAD      = getLCU(LCUIdx).m_costIntra;

    if (m_remainingCostIntra > 0.1 )
    {
      Double weightedBitsLeft = (m_bitsLeft*bitrateWindow+(m_bitsLeft-getLCU(LCUIdx).m_targetBitsLeft)*noOfLCUsLeft)/(Double)bitrateWindow;
      avgBits = Int( MAD*weightedBitsLeft/m_remainingCostIntra );
    }
    else
    {
      avgBits = Int( m_bitsLeft / m_LCULeft );
    }
    m_remainingCostIntra -= MAD;
  }
  else
  {
    Double totalWeight = 0;
    for ( Int i=LCUIdx; i<m_numberOfLCU; i++ )
    {
      totalWeight += m_LCUs[i].m_bitWeight;
    }
    Int realInfluenceLCU = min( g_RCLCUSmoothWindowSize, getLCULeft() );
    avgBits = (Int)( m_LCUs[LCUIdx].m_bitWeight - ( totalWeight - m_bitsLeft ) / realInfluenceLCU + 0.5 );
  }

  if ( avgBits < 1 )
  {
    avgBits = 1;
  }

  bpp = ( Double )avgBits/( Double )m_LCUs[ LCUIdx ].m_numberOfPixel;
  m_LCUs[ LCUIdx ].m_targetBits = avgBits;

  return bpp;
}

Double TEncRCPic::getLCUEstLambda( Double bpp )
{
  Int   LCUIdx = getLCUCoded();
  Double alpha;
  Double beta;
  if ( m_encRCSeq->getUseLCUSeparateModel() )
  {
    alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
    beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;
  }
  else
  {
    alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
    beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
  }

  Double estLambda = alpha * pow( bpp, beta );
  //for Lambda clip, picture level clip
  Double clipPicLambda = m_estPicLambda;

  //for Lambda clip, LCU level clip
  Double clipNeighbourLambda = -1.0;
  for ( Int i=LCUIdx - 1; i>=0; i-- )
  {
    if ( m_LCUs[i].m_lambda > 0 )
    {
      clipNeighbourLambda = m_LCUs[i].m_lambda;
      break;
    }
  }

  if ( clipNeighbourLambda > 0.0 )
  {
    estLambda = Clip3( clipNeighbourLambda * pow( 2.0, -1.0/3.0 ), clipNeighbourLambda * pow( 2.0, 1.0/3.0 ), estLambda );
  }

  if ( clipPicLambda > 0.0 )
  {
    estLambda = Clip3( clipPicLambda * pow( 2.0, -2.0/3.0 ), clipPicLambda * pow( 2.0, 2.0/3.0 ), estLambda );
  }
  else
  {
    estLambda = Clip3( 10.0, 1000.0, estLambda );
  }

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1;
  }
#if JVET_K0390_RATE_CTRL
  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = Double(int64_t(estLambda * (Double)LAMBDA_PREC + 0.5)) / (Double)LAMBDA_PREC;
#endif
  return estLambda;
}

Int TEncRCPic::getLCUEstQP( Double lambda, Int clipPicQP )
{
  Int LCUIdx = getLCUCoded();
  Int estQP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 );

  //for Lambda clip, LCU level clip
  Int clipNeighbourQP = g_RCInvalidQPValue;
  for ( Int i=LCUIdx - 1; i>=0; i-- )
  {
    if ( (getLCU(i)).m_QP > g_RCInvalidQPValue )
    {
      clipNeighbourQP = getLCU(i).m_QP;
      break;
    }
  }

  if ( clipNeighbourQP > g_RCInvalidQPValue )
  {
    estQP = Clip3( clipNeighbourQP - 1, clipNeighbourQP + 1, estQP );
  }

  estQP = Clip3( clipPicQP - 2, clipPicQP + 2, estQP );

  return estQP;
}

#if JVET_M0600_RATE_CTRL
Void TEncRCPic::updateAfterCTU(Int LCUIdx, Int bits, Int QP, Double lambda, Double skipRatio, Bool updateLCUParameter)
#else
Void TEncRCPic::updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter )
#endif
{
  m_LCUs[LCUIdx].m_actualBits = bits;
  m_LCUs[LCUIdx].m_QP         = QP;
  m_LCUs[LCUIdx].m_lambda     = lambda;
#if JVET_K0390_RATE_CTRL
  m_LCUs[LCUIdx].m_actualSSE = m_LCUs[LCUIdx].m_actualMSE * m_LCUs[LCUIdx].m_numberOfPixel;
#endif

  m_LCULeft--;
  m_bitsLeft   -= bits;
  m_pixelsLeft -= m_LCUs[LCUIdx].m_numberOfPixel;

  if ( !updateLCUParameter )
  {
    return;
  }

  if ( !m_encRCSeq->getUseLCUSeparateModel() )
  {
    return;
  }

  Double alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
  Double beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;

  Int LCUActualBits   = m_LCUs[LCUIdx].m_actualBits;
  Int LCUTotalPixels  = m_LCUs[LCUIdx].m_numberOfPixel;
  Double bpp         = ( Double )LCUActualBits/( Double )LCUTotalPixels;
  Double calLambda   = alpha * pow( bpp, beta );
  Double inputLambda = m_LCUs[LCUIdx].m_lambda;

  if( inputLambda < 0.01 || calLambda < 0.01 || bpp < 0.0001 )
  {
    alpha *= ( 1.0 - m_encRCSeq->getAlphaUpdate() / 2.0 );
    beta  *= ( 1.0 - m_encRCSeq->getBetaUpdate() / 2.0 );

    alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
    beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

    TRCParameter rcPara;
    rcPara.m_alpha = alpha;
    rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
    rcPara.m_skipRatio = skipRatio;
#endif
#if JVET_K0390_RATE_CTRL
    if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
    {
      rcPara.m_validPix = 0;
    }
    else
    {
      rcPara.m_validPix = LCUTotalPixels;
    }

    Double MSE = m_LCUs[LCUIdx].m_actualMSE;
    Double updatedK = bpp * inputLambda / MSE;
    Double updatedC = MSE / pow(bpp, -updatedK);
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;

    if (bpp > 0 && updatedK > 0.0001)
    {
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
    else
    {
      rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
#else
    m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif
    return;
  }

  calLambda = Clip3( inputLambda / 10.0, inputLambda * 10.0, calLambda );
  alpha += m_encRCSeq->getAlphaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * alpha;
  Double lnbpp = log( bpp );
  lnbpp = Clip3( -5.0, -0.1, lnbpp );
  beta  += m_encRCSeq->getBetaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * lnbpp;

  alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
  beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

  TRCParameter rcPara;
  rcPara.m_alpha = alpha;
  rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
  rcPara.m_skipRatio = skipRatio;
#endif
#if JVET_K0390_RATE_CTRL
  if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
  {
    rcPara.m_validPix = 0;
  }
  else
  {
    rcPara.m_validPix = LCUTotalPixels;
  }

  Double MSE = m_LCUs[LCUIdx].m_actualMSE;
  Double updatedK = bpp * inputLambda / MSE;
  Double updatedC = MSE / pow(bpp, -updatedK);
  rcPara.m_alpha = updatedC * updatedK;
  rcPara.m_beta = -updatedK - 1.0;

  if (bpp > 0 && updatedK > 0.0001)
  {
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
  else
  {
    rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
#else
  m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif
}

Double TEncRCPic::calAverageQP()
{
  Int totalQPs = 0;
  Int numTotalLCUs = 0;

  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_QP > 0 )
    {
      totalQPs += m_LCUs[i].m_QP;
      numTotalLCUs++;
    }
  }

  Double avgQP = 0.0;
  if ( numTotalLCUs == 0 )
  {
    avgQP = g_RCInvalidQPValue;
  }
  else
  {
    avgQP = ((Double)totalQPs) / ((Double)numTotalLCUs);
  }
  return avgQP;
}

Double TEncRCPic::calAverageLambda()
{
  Double totalLambdas = 0.0;
  Int numTotalLCUs = 0;
#if JVET_K0390_RATE_CTRL
  Double totalSSE = 0.0;
  Int totalPixels = 0;
#endif
  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_lambda > 0.01 )
    {
#if JVET_K0390_RATE_CTRL
      if (m_LCUs[i].m_QP > 0 || m_encRCSeq->getAdaptiveBits() != 1)
      {
        m_validPixelsInPic += m_LCUs[i].m_numberOfPixel;

        totalLambdas += log(m_LCUs[i].m_lambda);
        numTotalLCUs++;
      }
#else
      totalLambdas += log( m_LCUs[i].m_lambda );
      numTotalLCUs++;
#endif

#if JVET_K0390_RATE_CTRL
      if (m_LCUs[i].m_QP > 0 || m_encRCSeq->getAdaptiveBits() != 1)
      {
        totalSSE += m_LCUs[i].m_actualSSE;
        totalPixels += m_LCUs[i].m_numberOfPixel;
      }
#endif
    }
  }
#if JVET_K0390_RATE_CTRL
  setPicMSE(totalPixels > 0 ? totalSSE / (Double)totalPixels : 1.0); //1.0 is useless in the following process, just to make sure the divisor not be 0
#endif

  Double avgLambda;
  if( numTotalLCUs == 0 )
  {
    avgLambda = -1.0;
  }
  else
  {
    avgLambda = pow( 2.7183, totalLambdas / numTotalLCUs );
  }
  return avgLambda;
}


Void TEncRCPic::updateAfterPicture( Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType)
{
  m_picActualHeaderBits = actualHeaderBits;
  m_picActualBits       = actualTotalBits;
  if ( averageQP > 0.0 )
  {
    m_picQP             = Int( averageQP + 0.5 );
  }
  else
  {
    m_picQP             = g_RCInvalidQPValue;
  }
  m_picLambda           = averageLambda;

  Double alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
