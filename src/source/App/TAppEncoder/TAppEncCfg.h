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

/** \file     TAppEncCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#ifndef __TAPPENCCFG__
#define __TAPPENCCFG__

#include "TLibCommon/CommonDef.h"
#include "Utilities/program_options_lite.h"

#include "TLibEncoder/TEncCfg.h"
#if EXTENSION_360_VIDEO
#include "TAppEncHelper360/TExt360AppEncCfg.h"
#endif
#include <sstream>
#include <vector>

namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class TAppEncCfg
{
public:
  template <class T>
  struct OptionalValue
  {
    Bool bPresent;
    T    value;
    OptionalValue() : bPresent(false), value() { }
  };

protected:
  // file I/O
  std::string m_inputFileName;                                ///< source file name
  std::string m_bitstreamFileName;                            ///< output bitstream file
  std::string m_reconFileName;                                ///< output reconstruction file
#if SHUTTER_INTERVAL_SEI_PROCESSING
  Bool        m_ShutterFilterEnable;                          ///< enable Pre-Filtering with Shutter Interval SEI
  std::string m_shutterIntervalPreFileName;                   ///< output Pre-Filtering video
#endif

  // Lambda modifiers
  Double    m_adLambdaModifier[ MAX_TLAYER ];                 ///< Lambda modifier array for each temporal layer
  std::vector<Double> m_adIntraLambdaModifier;                ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  Double    m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  // source specification
  Int       m_iFrameRate;                                     ///< source frame-rates (Hz)
  UInt      m_FrameSkip;                                      ///< number of skipped frames from the beginning
  UInt      m_temporalSubsampleRatio;                         ///< temporal subsample ratio, 2 means code every two frames
  Int       m_sourceWidth;                                    ///< source width in pixel
  Int       m_sourceHeight;                                   ///< source height in pixel (when interlaced = field height)
  Int       m_inputFileWidth;                                 ///< width of image in input file  (this is equivalent to sourceWidth,  if sourceWidth  is not subsequently altered due to padding)
  Int       m_inputFileHeight;                                ///< height of image in input file (this is equivalent to sourceHeight, if sourceHeight is not subsequently altered due to padding)

  Int       m_sourceHeightOrg;                                ///< original source height in pixel (when interlaced = frame height)

  Bool      m_isField;                                        ///< enable field coding
  Bool      m_isTopFieldFirst;
  Bool      m_bEfficientFieldIRAPEnabled;                     ///< enable an efficient field IRAP structure.
  Bool      m_bHarmonizeGopFirstFieldCoupleEnabled;

  Int       m_conformanceWindowMode;
  Int       m_confWinLeft;
  Int       m_confWinRight;
  Int       m_confWinTop;
  Int       m_confWinBottom;
  Int       m_sourcePadding[2];                               ///< number of padded pixels for width and height
  Int       m_framesToBeEncoded;                              ///< number of encoded frames
  Bool      m_AccessUnitDelimiter;                            ///< add Access Unit Delimiter NAL units
  InputColourSpaceConversion m_inputColourSpaceConvert;       ///< colour space conversion to apply to input video
  Bool      m_snrInternalColourSpace;                       ///< if true, then no colour space conversion is applied for snr calculation, otherwise inverse of input is applied.
  Bool      m_outputInternalColourSpace;                    ///< if true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.
  ChromaFormat m_InputChromaFormatIDC;

  Bool      m_printMSEBasedSequencePSNR;
  Bool      m_printHexPsnr;
  Bool      m_printFrameMSE;
  Bool      m_printSequenceMSE;
  Bool      m_printMSSSIM;

  Bool      m_bXPSNREnableFlag;                              ///< xPSNR enable flag
  Double    m_dXPSNRWeight[MAX_NUM_COMPONENT];               ///< xPSNR per component weights

  Bool      m_cabacZeroWordPaddingEnabled;
  Bool      m_bClipInputVideoToRec709Range;
  Bool      m_bClipOutputVideoToRec709Range;

  // profile/level
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
  UInt          m_bitDepthConstraint;
  ChromaFormat  m_chromaFormatConstraint;
  Bool          m_intraConstraintFlag;
  Bool          m_onePictureOnlyConstraintFlag;
  Bool          m_lowerBitRateConstraintFlag;
  Bool m_progressiveSourceFlag;
  Bool m_interlacedSourceFlag;
  Bool m_nonPackedConstraintFlag;
  Bool m_frameOnlyConstraintFlag;

  // coding structure
  Int       m_iIntraPeriod;                                   ///< period of I-slice (random access period)
  Int       m_iDecodingRefreshType;                           ///< random access type
  Int       m_iGOPSize;                                       ///< GOP size of hierarchical structure
  Bool      m_bReWriteParamSetsFlag;                          ///< Flag to enable rewriting of parameter sets at random access points
  Int       m_extraRPSs;                                      ///< extra RPSs added to handle CRA
  GOPEntry  m_GOPList[MAX_GOP];                               ///< the coding structure entries from the config file
  Int       m_numReorderPics[MAX_TLAYER];                     ///< total number of reorder pictures
  Int       m_maxDecPicBuffering[MAX_TLAYER];                 ///< total number of pictures in the decoded picture buffer
  Bool      m_crossComponentPredictionEnabledFlag;            ///< flag enabling the use of cross-component prediction
  Bool      m_reconBasedCrossCPredictionEstimate;             ///< causes the alpha calculation in encoder search to be based on the decoded residual rather than the pre-transform encoder-side residual
  UInt      m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];       ///< number of bits for the upward bit shift operation on the decoded SAO offsets
  Bool      m_useTransformSkip;                               ///< flag for enabling intra transform skipping
  Bool      m_useTransformSkipFast;                           ///< flag for enabling fast intra transform skipping
  UInt      m_log2MaxTransformSkipBlockSize;                  ///< transform-skip maximum size (minimum of 2)
  Bool      m_transformSkipRotationEnabledFlag;               ///< control flag for transform-skip/transquant-bypass residual rotation
  Bool      m_transformSkipContextEnabledFlag;                ///< control flag for transform-skip/transquant-bypass single significance map context
  Bool      m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];///< control flags for residual DPCM
  Bool      m_enableAMP;
  Bool      m_persistentRiceAdaptationEnabledFlag;            ///< control flag for Golomb-Rice parameter adaptation over each slice
  Bool      m_cabacBypassAlignmentEnabledFlag;

  // coding quality
  OptionalValue<UInt> m_qpIncrementAtSourceFrame;             ///< Optional source frame number at which all subsequent frames are to use an increased internal QP.
  Int       m_iQP;                                            ///< QP value of key-picture (integer)
  Int       m_intraQPOffset;                                  ///< QP offset for intra slice (integer)
  Bool      m_lambdaFromQPEnable;                             ///< enable flag for QP:lambda fix
  std::string m_dQPFileName;                                  ///< QP offset for each slice (initialized from external file)
  Int*      m_aidQP;                                          ///< array of slice QP values
  Int       m_iMaxDeltaQP;                                    ///< max. |delta QP|
  UInt      m_uiDeltaQpRD;                                    ///< dQP range for multi-pass slice QP optimization
  Int       m_iMaxCuDQPDepth;                                 ///< Max. depth for a minimum CuDQPSize (0:default)
  Int       m_diffCuChromaQpOffsetDepth;                      ///< If negative, then do not apply chroma qp offsets.
  Bool      m_bFastDeltaQP;                                   ///< Fast Delta QP (false:default)

  Int       m_cbQpOffset;                                     ///< Chroma Cb QP Offset (0:default)
  Int       m_crQpOffset;                                     ///< Chroma Cr QP Offset (0:default)
  WCGChromaQPControl m_wcgChromaQpControl;                    ///< Wide-colour-gamut chroma QP control.
  UInt      m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  Int       m_sliceChromaQpOffsetIntraOrPeriodic[2/*Cb,Cr*/]; ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping;      ///< mapping from luma level to Delta QP.
#if ADAPTIVE_QP_SELECTION
  Bool      m_bUseAdaptQpSelect;
#endif
#if JVET_V0078
  Bool      m_bSmoothQPReductionEnable;
  Double    m_dSmoothQPReductionThreshold;
  Double    m_dSmoothQPReductionModelScale;
  Double    m_dSmoothQPReductionModelOffset;
  Int       m_iSmoothQPReductionPeriodicity;
  Int       m_iSmoothQPReductionLimit;
#endif
  TComSEIMasteringDisplay m_masteringDisplay;

  Bool      m_bUseAdaptiveQP;                                 ///< Flag for enabling QP adaptation based on a psycho-visual model
  Int       m_iQPAdaptationRange;                             ///< dQP range by QP adaptation

  Int       m_maxTempLayer;                                  ///< Max temporal layer

  // coding unit (CU) definition
  // TODO: Remove MaxCUWidth/MaxCUHeight and replace with MaxCUSize.
  UInt      m_uiMaxCUWidth;                                   ///< max. CU width in pixel
  UInt      m_uiMaxCUHeight;                                  ///< max. CU height in pixel
  UInt      m_uiMaxCUDepth;                                   ///< max. CU depth (as specified by command line)
  UInt      m_uiMaxTotalCUDepth;                              ///< max. total CU depth - includes depth of transform-block structure
  UInt      m_uiLog2DiffMaxMinCodingBlockSize;                ///< difference between largest and smallest CU depth

  // transfom unit (TU) definition
  UInt      m_uiQuadtreeTULog2MaxSize;
  UInt      m_uiQuadtreeTULog2MinSize;

  UInt      m_uiQuadtreeTUMaxDepthInter;
  UInt      m_uiQuadtreeTUMaxDepthIntra;

  // coding tools (bit-depth)
  Int       m_inputBitDepth   [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  Int       m_outputBitDepth  [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  Int       m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of input samples after MSB extension
  Int       m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth codec operates at (input/output files will be converted)
  Bool      m_extendedPrecisionProcessingFlag;
  Bool      m_highPrecisionOffsetsEnabledFlag;

  //coding tools (chroma format)
  ChromaFormat m_chromaFormatIDC;

  // coding tools (PCM bit-depth)
  Bool      m_bPCMInputBitDepthFlag;                          ///< 0: PCM bit-depth is internal bit-depth. 1: PCM bit-depth is input bit-depth.

  // coding tool (SAO)
  Bool      m_bUseSAO;
  Bool      m_bTestSAODisableAtPictureLevel;
  Double    m_saoEncodingRate;                                ///< When >0 SAO early picture termination is enabled for luma and chroma
  Double    m_saoEncodingRateChroma;                          ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  Int       m_maxNumOffsetsPerPic;                            ///< SAO maximun number of offset per picture
  Bool      m_saoCtuBoundary;                                 ///< SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
  Bool      m_resetEncoderStateAfterIRAP;                     ///< When true, encoder state will be reset following an IRAP.
  // coding tools (loop filter)
  Bool      m_bLoopFilterDisable;                             ///< flag for using deblocking filter
  Bool      m_loopFilterOffsetInPPS;                          ///< offset for deblocking filter in 0 = slice header, 1 = PPS
  Int       m_loopFilterBetaOffsetDiv2;                       ///< beta offset for deblocking filter
  Int       m_loopFilterTcOffsetDiv2;                         ///< tc offset for deblocking filter
  Int       m_deblockingFilterMetric;                         ///< blockiness metric in encoder
  // coding tools (PCM)
  Bool      m_usePCM;                                         ///< flag for using IPCM
  UInt      m_pcmLog2MaxSize;                                 ///< log2 of maximum PCM block size
  UInt      m_uiPCMLog2MinSize;                               ///< log2 of minimum PCM block size
  Bool      m_bPCMFilterDisableFlag;                          ///< PCM filter disable flag
  Bool      m_enableIntraReferenceSmoothing;                  ///< flag for enabling(default)/disabling intra reference smoothing/filtering

  // coding tools (encoder-only parameters)
  Bool      m_bUseASR;                                        ///< flag for using adaptive motion search range
  Bool      m_bUseHADME;                                      ///< flag for using HAD in sub-pel ME
  Bool      m_useRDOQ;                                        ///< flag for using RD optimized quantization
  Bool      m_useRDOQTS;                                      ///< flag for using RD optimized quantization for transform skip
  Bool      m_useSelectiveRDOQ;                               ///< flag for using selective RDOQ
  Int       m_rdPenalty;                                      ///< RD-penalty for 32x32 TU for intra in non-intra slices (0: no RD-penalty, 1: RD-penalty, 2: maximum RD-penalty)
  Bool      m_bDisableIntraPUsInInterSlices;                  ///< Flag for disabling intra predicted PUs in inter slices.
  MESearchMethod m_motionEstimationSearchMethod;
  Bool      m_bRestrictMESampling;                            ///< Restrict sampling for the Selective ME
  Int       m_iSearchRange;                                   ///< ME search range
  Int       m_bipredSearchRange;                              ///< ME search range for bipred refinement
  Int       m_minSearchWindow;                                ///< ME minimum search window size for the Adaptive Window ME
  Bool      m_bClipForBiPredMeEnabled;                        ///< Enables clipping for Bi-Pred ME.
  Bool      m_bFastMEAssumingSmootherMVEnabled;               ///< Enables fast ME assuming a smoother MV.
  FastInterSearchMode m_fastInterSearchMode;                  ///< Parameter that controls fast encoder settings
  Bool      m_bUseEarlyCU;                                    ///< flag for using Early CU setting
  Bool      m_useFastDecisionForMerge;                        ///< flag for using Fast Decision Merge RD-Cost
  Bool      m_bUseCbfFastMode;                                ///< flag for using Cbf Fast PU Mode Decision
  Bool      m_useEarlySkipDetection;                          ///< flag for using Early SKIP Detection
  SliceConstraint m_sliceMode;
  Int             m_sliceArgument;                            ///< argument according to selected slice mode
  SliceConstraint m_sliceSegmentMode;
  Int             m_sliceSegmentArgument;                     ///< argument according to selected slice segment mode

  Bool      m_bLFCrossSliceBoundaryFlag;  ///< 1: filter across slice boundaries 0: do not filter across slice boundaries
  Bool      m_bLFCrossTileBoundaryFlag;   ///< 1: filter across tile boundaries  0: do not filter across tile boundaries
  Bool      m_tileUniformSpacingFlag;
  Int       m_numTileColumnsMinus1;
  Int       m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;
  Bool      m_entropyCodingSyncEnabledFlag;

  Bool      m_bUseConstrainedIntraPred;                       ///< flag for using constrained intra prediction
  Bool      m_bFastUDIUseMPMEnabled;
  Bool      m_bFastMEForGenBLowDelayEnabled;
  Bool      m_bUseBLambdaForNonKeyLowDelayPictures;

  HashType  m_decodedPictureHashSEIType;                      ///< Checksum mode for decoded picture hash SEI message
  Bool      m_recoveryPointSEIEnabled;
  Bool      m_bufferingPeriodSEIEnabled;
  Bool      m_pictureTimingSEIEnabled;
  Bool      m_toneMappingInfoSEIEnabled;
  Bool      m_chromaResamplingFilterSEIenabled;
  Int       m_chromaResamplingHorFilterIdc;
  Int       m_chromaResamplingVerFilterIdc;
  Int       m_toneMapId;
  Bool      m_toneMapCancelFlag;
