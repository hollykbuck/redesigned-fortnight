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

/** \file     TEncRateCtrl.h
    \brief    Rate control manager class
*/

#ifndef __TENCRATECTRL__
#define __TENCRATECTRL__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "../TLibCommon/CommonDef.h"
#include "../TLibCommon/TComDataCU.h"

#include <vector>
#include <algorithm>

using namespace std;

//! \ingroup TLibEncoder
//! \{

#include "../TLibEncoder/TEncCfg.h"
#include <list>
#include <cassert>

const Int g_RCInvalidQPValue = -999;
#if JVET_Y0105_SW_AND_QDF
const int g_RCSmoothWindowSizeAlpha = 20;
const int g_RCSmoothWindowSizeBeta = 60;
#else
const int g_RCSmoothWindowSize = 40;
#endif
const Int g_RCMaxPicListSize = 32;
const Double g_RCWeightPicTargetBitInGOP    = 0.9;
const Double g_RCWeightPicRargetBitInBuffer = 1.0 - g_RCWeightPicTargetBitInGOP;
const Int g_RCIterationNum = 20;
const Double g_RCWeightHistoryLambda = 0.5;
const Double g_RCWeightCurrentLambda = 1.0 - g_RCWeightHistoryLambda;
const Int g_RCLCUSmoothWindowSize = 4;
const Double g_RCAlphaMinValue = 0.05;
const Double g_RCAlphaMaxValue = 500.0;
const Double g_RCBetaMinValue  = -3.0;
const Double g_RCBetaMaxValue  = -0.1;
#if JVET_K0390_RATE_CTRL
const Int LAMBDA_PREC = 1000000;
#endif

#define ALPHA     6.7542;
#define BETA1     1.2517
#define BETA2     1.7860

struct TRCLCU
{
  Int m_actualBits;
  Int m_QP;     // QP of skip mode is set to g_RCInvalidQPValue
  Int m_targetBits;
  Double m_lambda;
  Double m_bitWeight;
  Int m_numberOfPixel;
  Double m_costIntra;
  Int m_targetBitsLeft;
#if JVET_K0390_RATE_CTRL
  Double m_actualSSE;
  Double m_actualMSE;
#endif
};

struct TRCParameter
{
  Double m_alpha;
  Double m_beta;
#if JVET_K0390_RATE_CTRL
  Int    m_validPix;
#endif
#if JVET_M0600_RATE_CTRL
  Double m_skipRatio;
#endif
};

class TEncRCSeq
{
public:
  TEncRCSeq();
  ~TEncRCSeq();

public:
#if JVET_Y0105_SW_AND_QDF
  Void create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int intraPeriod, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit );
#else
  Void create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit );
#endif
  Void destroy();
  Void initBitsRatio( Int bitsRatio[] );
  Void initGOPID2Level( Int GOPID2Level[] );
  Void initPicPara( TRCParameter* picPara  = NULL );    // NULL to initial with default value
  Void initLCUPara( TRCParameter** LCUPara = NULL );    // NULL to initial with default value
  Void updateAfterPic ( Int bits );
  Void setAllBitRatio( Double basicLambda, Double* equaCoeffA, Double* equaCoeffB );

public:
  Int  getTotalFrames()                 { return m_totalFrames; }
  Int  getTargetRate()                  { return m_targetRate; }
  Int  getFrameRate()                   { return m_frameRate; }
  Int  getGOPSize()                     { return m_GOPSize; }
#if JVET_Y0105_SW_AND_QDF
  Int  getIntraPeriod()                 { return m_intraPeriod; }
#endif
  Int  getPicWidth()                    { return m_picWidth; }
  Int  getPicHeight()                   { return m_picHeight; }
  Int  getLCUWidth()                    { return m_LCUWidth; }
  Int  getLCUHeight()                   { return m_LCUHeight; }
  Int  getNumberOfLevel()               { return m_numberOfLevel; }
  Int  getAverageBits()                 { return m_averageBits; }
  Int  getLeftAverageBits()             { assert( m_framesLeft > 0 ); return (Int)(m_bitsLeft / m_framesLeft); }
  Bool getUseLCUSeparateModel()         { return m_useLCUSeparateModel; }

  Int  getNumPixel()                    { return m_numberOfPixel; }
  Int64  getTargetBits()                { return m_targetBits; }
  Int  getNumberOfLCU()                 { return m_numberOfLCU; }
  Int* getBitRatio()                    { return m_bitsRatio; }
  Int  getBitRatio( Int idx )           { assert( idx<m_GOPSize); return m_bitsRatio[idx]; }
  Int* getGOPID2Level()                 { return m_GOPID2Level; }
  Int  getGOPID2Level( Int ID )         { assert( ID < m_GOPSize ); return m_GOPID2Level[ID]; }
  TRCParameter*  getPicPara()                                   { return m_picPara; }
  TRCParameter   getPicPara( Int level )                        { assert( level < m_numberOfLevel ); return m_picPara[level]; }
  Void           setPicPara( Int level, TRCParameter para )     { assert( level < m_numberOfLevel ); m_picPara[level] = para; }
  TRCParameter** getLCUPara()                                   { return m_LCUPara; }
  TRCParameter*  getLCUPara( Int level )                        { assert( level < m_numberOfLevel ); return m_LCUPara[level]; }
  TRCParameter   getLCUPara( Int level, Int LCUIdx )            { assert( LCUIdx  < m_numberOfLCU ); return getLCUPara(level)[LCUIdx]; }
  Void           setLCUPara( Int level, Int LCUIdx, TRCParameter para ) { assert( level < m_numberOfLevel ); assert( LCUIdx  < m_numberOfLCU ); m_LCUPara[level][LCUIdx] = para; }

  Int  getFramesLeft()                  { return m_framesLeft; }
  Int64  getBitsLeft()                  { return m_bitsLeft; }

  Double getSeqBpp()                    { return m_seqTargetBpp; }
  Double getAlphaUpdate()               { return m_alphaUpdate; }
  Double getBetaUpdate()                { return m_betaUpdate; }

  Int    getAdaptiveBits()              { return m_adaptiveBit;  }
  Double getLastLambda()                { return m_lastLambda;   }
  Void   setLastLambda( Double lamdba ) { m_lastLambda = lamdba; }

private:
  Int m_totalFrames;
  Int m_targetRate;
  Int m_frameRate;
  Int m_GOPSize;
#if JVET_Y0105_SW_AND_QDF
  Int m_intraPeriod;
#endif
  Int m_picWidth;
  Int m_picHeight;
  Int m_LCUWidth;
  Int m_LCUHeight;
  Int m_numberOfLevel;
  Int m_averageBits;

  Int m_numberOfPixel;
  Int64 m_targetBits;
  Int m_numberOfLCU;
  Int* m_bitsRatio;
  Int* m_GOPID2Level;
  TRCParameter*  m_picPara;
  TRCParameter** m_LCUPara;

  Int m_framesLeft;
  Int64 m_bitsLeft;
  Double m_seqTargetBpp;
  Double m_alphaUpdate;
  Double m_betaUpdate;
  Bool m_useLCUSeparateModel;

  Int m_adaptiveBit;
  Double m_lastLambda;
};

class TEncRCGOP
{
public:
  TEncRCGOP();
  ~TEncRCGOP();

public:
#if JVET_Y0105_SW_AND_QDF
  Void create( TEncRCSeq* encRCSeq, Int numPic, bool useAdaptiveBitsRatio );
#else
  Void create( TEncRCSeq* encRCSeq, Int numPic );
#endif
  Void destroy();
  Void updateAfterPicture( Int bitsCost );

private:
  Int  xEstGOPTargetBits( TEncRCSeq* encRCSeq, Int GOPSize );
  Void   xCalEquaCoeff( TEncRCSeq* encRCSeq, Double* lambdaRatio, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize );
#if JVET_K0390_RATE_CTRL
  Double xSolveEqua(TEncRCSeq* encRCSeq, Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize);
#else
  Double xSolveEqua( Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize );
#endif
public:
  TEncRCSeq* getEncRCSeq()        { return m_encRCSeq; }
  Int  getNumPic()                { return m_numPic;}
  Int  getTargetBits()            { return m_targetBits; }
  Int  getPicLeft()               { return m_picLeft; }
  Int  getBitsLeft()              { return m_bitsLeft; }
  Int  getTargetBitInGOP( Int i ) { return m_picTargetBitInGOP[i]; }

private:
  TEncRCSeq* m_encRCSeq;
  Int* m_picTargetBitInGOP;
  Int m_numPic;
  Int m_targetBits;
  Int m_picLeft;
  Int m_bitsLeft;
};

class TEncRCPic
{
public:
  TEncRCPic();
  ~TEncRCPic();

public:
  Void create( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP, Int frameLevel, list<TEncRCPic*>& listPreviousPictures );
  Void destroy();

  Int    estimatePicQP    ( Double lambda, list<TEncRCPic*>& listPreviousPictures );
  Int    getRefineBitsForIntra(Int orgBits);
  Double calculateLambdaIntra(Double alpha, Double beta, Double MADPerPixel, Double bitsPerPixel);
  Double estimatePicLambda( list<TEncRCPic*>& listPreviousPictures, SliceType eSliceType);

  Void   updateAlphaBetaIntra(Double *alpha, Double *beta);

  Double getLCUTargetBpp(SliceType eSliceType);
  Double getLCUEstLambdaAndQP(Double bpp, Int clipPicQP, Int *estQP);
  Double getLCUEstLambda( Double bpp );
  Int    getLCUEstQP( Double lambda, Int clipPicQP );
#if JVET_M0600_RATE_CTRL
  void updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, Double skipRatio, Bool updateLCUParameter = true);
#else
  Void updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter = true );
#endif
  Void updateAfterPicture( Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType);

  Void addToPictureLsit( list<TEncRCPic*>& listPreviousPictures );
  Double calAverageQP();
  Double calAverageLambda();

private:
  Int xEstPicTargetBits( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP );
  Int xEstPicHeaderBits( list<TEncRCPic*>& listPreviousPictures, Int frameLevel );
  Int xEstPicLowerBound( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP );

public:
  TEncRCSeq*      getRCSequence()                         { return m_encRCSeq; }
  TEncRCGOP*      getRCGOP()                              { return m_encRCGOP; }

  Int  getFrameLevel()                                    { return m_frameLevel; }
  Int  getNumberOfPixel()                                 { return m_numberOfPixel; }
  Int  getNumberOfLCU()                                   { return m_numberOfLCU; }
  Int  getTargetBits()                                    { return m_targetBits; }
  Int  getEstHeaderBits()                                 { return m_estHeaderBits; }
  Int  getLCULeft()                                       { return m_LCULeft; }
  Int  getBitsLeft()                                      { return m_bitsLeft; }
  Int  getPixelsLeft()                                    { return m_pixelsLeft; }
