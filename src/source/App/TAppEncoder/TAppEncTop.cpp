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
  m_cTEncTop.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
  m_cTEncTop.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_cTEncTop.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_cTEncTop.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
  m_cTEncTop.setLumaLevelToDeltaQPControls                        ( m_lumaLevelToDeltaQPMapping );
  m_cTEncTop.setDeltaQpRD( (m_costMode==COST_LOSSLESS_CODING) ? 0 : m_uiDeltaQpRD );
  m_cTEncTop.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_cTEncTop.setUseASR                                            ( m_bUseASR      );
  m_cTEncTop.setUseHADME                                          ( m_bUseHADME    );
  m_cTEncTop.setdQPs                                              ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                                           ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                                         ( m_useRDOQTS   );
  m_cTEncTop.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
  m_cTEncTop.setRDpenalty                                         ( m_rdPenalty );
  m_cTEncTop.setMaxCUWidth                                        ( m_uiMaxCUWidth );
  m_cTEncTop.setMaxCUHeight                                       ( m_uiMaxCUHeight );
  m_cTEncTop.setMaxTotalCUDepth                                   ( m_uiMaxTotalCUDepth );
  m_cTEncTop.setLog2DiffMaxMinCodingBlockSize                     ( m_uiLog2DiffMaxMinCodingBlockSize );
  m_cTEncTop.setQuadtreeTULog2MaxSize                             ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize                             ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter                           ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra                           ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_cTEncTop.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cTEncTop.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cTEncTop.setCrossComponentPredictionEnabledFlag               ( m_crossComponentPredictionEnabledFlag );
  m_cTEncTop.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cTEncTop.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_LUMA  , m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA]   );
  m_cTEncTop.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  m_cTEncTop.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_cTEncTop.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_cTEncTop.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_cTEncTop.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_cTEncTop.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_cTEncTop.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_cTEncTop.setRdpcmEnabledFlag                                ( RDPCMSignallingMode(signallingModeIndex), m_rdpcmEnabledFlag[signallingModeIndex]);
  }
  m_cTEncTop.setUseConstrainedIntraPred                           ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_cTEncTop.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_cTEncTop.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_cTEncTop.setPCMLog2MinSize                                    ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                                            ( m_usePCM );

  // set internal bit-depth and constants
  for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_cTEncTop.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_cTEncTop.setPCMBitDepth((ChannelType)channelType, m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[channelType] : m_internalBitDepth[channelType]);
#if JVET_X0048_X0103_FILM_GRAIN
    m_cTEncTop.setBitDepthInput((ChannelType)channelType, m_inputBitDepth[channelType]);
#endif
  }

  m_cTEncTop.setPCMLog2MaxSize                                    ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand                                   ( m_maxNumMergeCand );


  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                                             ( m_useWeightedPred     );
  m_cTEncTop.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2                      ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode                                         ( m_sliceMode );
  m_cTEncTop.setSliceArgument                                     ( m_sliceArgument );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode                                  ( m_sliceSegmentMode );
  m_cTEncTop.setSliceSegmentArgument                              ( m_sliceSegmentArgument );

  if(m_sliceMode == NO_SLICES )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag                          ( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO                                            ( m_bUseSAO );
  m_cTEncTop.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_cTEncTop.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_cTEncTop.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_cTEncTop.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoCtuBoundary                                    ( m_saoCtuBoundary);
  m_cTEncTop.setResetEncoderStateAfterIRAP                        ( m_resetEncoderStateAfterIRAP );
  m_cTEncTop.setPCMInputBitDepthFlag                              ( m_bPCMInputBitDepthFlag);
  m_cTEncTop.setPCMFilterDisableFlag                              ( m_bPCMFilterDisableFlag);

  m_cTEncTop.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );
  m_cTEncTop.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
  m_cTEncTop.setRecoveryPointSEIEnabled                           ( m_recoveryPointSEIEnabled );
