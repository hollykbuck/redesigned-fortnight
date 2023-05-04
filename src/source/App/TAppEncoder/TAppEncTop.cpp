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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>

#include "TAppEncTop.h"
#include "TLibEncoder/TEncTemporalFilter.h"
#include "TLibEncoder/AnnexBwrite.h"

#if EXTENSION_360_VIDEO
#include "TAppEncHelper360/TExt360AppEncTop.h"
#endif

using namespace std;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
}

TAppEncTop::~TAppEncTop()
{
}

Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;

  vps.setMaxTLayers                                               ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                                                ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                                         ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering                                     ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile                                           ( m_profile);
  m_cTEncTop.setLevel                                             ( m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag                           ( m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cTEncTop.setBitDepthConstraintValue                           ( m_bitDepthConstraint );
  m_cTEncTop.setChromaFormatConstraintValue                       ( m_chromaFormatConstraint );
  m_cTEncTop.setIntraConstraintFlag                               ( m_intraConstraintFlag );
  m_cTEncTop.setOnePictureOnlyConstraintFlag                      ( m_onePictureOnlyConstraintFlag );
  m_cTEncTop.setLowerBitRateConstraintFlag                        ( m_lowerBitRateConstraintFlag );

  m_cTEncTop.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cTEncTop.setPrintHexPsnr                                      ( m_printHexPsnr);
  m_cTEncTop.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cTEncTop.setPrintSequenceMSE                                  ( m_printSequenceMSE);
  m_cTEncTop.setPrintMSSSIM                                       ( m_printMSSSIM );

  m_cTEncTop.setXPSNREnableFlag                                   ( m_bXPSNREnableFlag);
  for (Int id = 0 ; id < MAX_NUM_COMPONENT; id++)
  {
    m_cTEncTop.setXPSNRWeight                                     ( m_dXPSNRWeight[id], ComponentID(id));
  }

#if SHUTTER_INTERVAL_SEI_PROCESSING
  m_cTEncTop.setShutterFilterFlag                                 ( m_ShutterFilterEnable );
#endif

  m_cTEncTop.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_cTEncTop.setFrameRate                                         ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                                         ( m_FrameSkip );
  m_cTEncTop.setTemporalSubsampleRatio                            ( m_temporalSubsampleRatio );
  m_cTEncTop.setSourceWidth                                       ( m_sourceWidth );
  m_cTEncTop.setSourceHeight                                      ( m_sourceHeight );
  m_cTEncTop.setConformanceWindow                                 ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTop.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                                       ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                                           ( m_iGOPSize );
  m_cTEncTop.setReWriteParamSetsFlag                              ( m_bReWriteParamSetsFlag );
  m_cTEncTop.setGopList                                           ( m_GOPList );
  m_cTEncTop.setExtraRPSs                                         ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_cTEncTop.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_cTEncTop.setQP                                                ( m_iQP );

  m_cTEncTop.setIntraQPOffset                                     ( m_intraQPOffset );
  m_cTEncTop.setLambdaFromQPEnable                                ( m_lambdaFromQPEnable );
  m_cTEncTop.setSourcePadding                                     ( m_sourcePadding );

  m_cTEncTop.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );

  m_cTEncTop.setMaxTempLayer                                      ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterMetric                            ( m_deblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_cTEncTop.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_cTEncTop.setSearchRange                                       ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_cTEncTop.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_cTEncTop.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_cTEncTop.setMinSearchWindow                                   ( m_minSearchWindow );
  m_cTEncTop.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                                     ( m_iMaxCuDQPDepth  );
  m_cTEncTop.setDiffCuChromaQpOffsetDepth                         ( m_diffCuChromaQpOffsetDepth );
  m_cTEncTop.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset                                  ( m_crQpOffset  );
  m_cTEncTop.setWCGChromaQpControl                                ( m_wcgChromaQpControl );
  m_cTEncTop.setSliceChromaOffsetQpIntraOrPeriodic                ( m_sliceChromaQpOffsetPeriodicity, m_sliceChromaQpOffsetIntraOrPeriodic );
  m_cTEncTop.setChromaFormatIdc                                   ( m_chromaFormatIDC  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect                                  ( m_bUseAdaptQpSelect   );
#endif
#if  JVET_V0078
  m_cTEncTop.setSmoothQPReductionEnable                           (m_bSmoothQPReductionEnable);
  m_cTEncTop.setSmoothQPReductionThreshold                        (m_dSmoothQPReductionThreshold);
  m_cTEncTop.setSmoothQPReductionModelScale                       (m_dSmoothQPReductionModelScale);
  m_cTEncTop.setSmoothQPReductionModelOffset                      (m_dSmoothQPReductionModelOffset);
  m_cTEncTop.setSmoothQPReductionLimit                            (m_iSmoothQPReductionLimit);
  m_cTEncTop.setSmoothQPReductionPeriodicity                      (m_iSmoothQPReductionPeriodicity);
#endif

  m_cTEncTop.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
