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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>
#include <map>

#include "CommonDef.h"
#include "libmd5/MD5.h"

//! \ingroup TLibCommon
//! \{
class TComSPS;

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    PAN_SCAN_RECT                        = 2,   // TODO: add encoder command line control to create these messages
    FILLER_PAYLOAD                       = 3,   // TODO: add encoder command line control to create these messages
    USER_DATA_REGISTERED_ITU_T_T35       = 4,   // TODO: add encoder command line control to create these messages
    USER_DATA_UNREGISTERED               = 5,   // TODO: add encoder command line control to create these messages
    RECOVERY_POINT                       = 6,
    SCENE_INFO                           = 9,   // TODO: add encoder command line control to create these messages
    PICTURE_SNAPSHOT                     = 15,  // TODO: add encoder command line control to create these messages
    PROGRESSIVE_REFINEMENT_SEGMENT_START = 16,  // TODO: add encoder command line control to create these messages
    PROGRESSIVE_REFINEMENT_SEGMENT_END   = 17,  // TODO: add encoder command line control to create these messages
    FILM_GRAIN_CHARACTERISTICS           = 19,  // TODO: add encoder command line control to create these messages
    POST_FILTER_HINT                     = 22,  // TODO: add encoder command line control to create these messages
    TONE_MAPPING_INFO                    = 23,
    FRAME_PACKING                        = 45,
    DISPLAY_ORIENTATION                  = 47,
    GREEN_METADATA                       = 56,
    SOP_DESCRIPTION                      = 128,
    ACTIVE_PARAMETER_SETS                = 129,
    DECODING_UNIT_INFO                   = 130,
    TEMPORAL_LEVEL0_INDEX                = 131,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    REGION_REFRESH_INFO                  = 134,
    NO_DISPLAY                           = 135,
    TIME_CODE                            = 136,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    SEGM_RECT_FRAME_PACKING              = 138,
    TEMP_MOTION_CONSTRAINED_TILE_SETS    = 139,
    CHROMA_RESAMPLING_FILTER_HINT        = 140,
    KNEE_FUNCTION_INFO                   = 141,
    COLOUR_REMAPPING_INFO                = 142,
    DEINTERLACE_FIELD_IDENTIFICATION     = 143, // TODO: add encoder command line control to create these messages
    CONTENT_LIGHT_LEVEL_INFO             = 144, // TODO: add encoder command line control to create these messages
    DEPENDENT_RAP_INDICATION             = 145, // TODO: add encoder command line control to create these messages
    CODED_REGION_COMPLETION              = 146, // TODO: add encoder command line control to create these messages
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148, // TODO: add encoder command line control to create these messages
    CONTENT_COLOUR_VOLUME                = 149, 
    EQUIRECTANGULAR_PROJECTION           = 150,
    SPHERE_ROTATION                      = 154,
    OMNI_VIEWPORT                        = 156,
    CUBEMAP_PROJECTION                   = 151,
    FISHEYE_VIDEO_INFO                   = 152,
    REGION_WISE_PACKING                  = 155, 
    REGIONAL_NESTING                     = 157,
#if MCTS_EXTRACTION
    MCTS_EXTRACTION_INFO_SET             = 158,
#endif
#if JCTVC_AD0021_SEI_MANIFEST
    SEI_MANIFEST                         = 200,
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    SEI_PREFIX_INDICATION                = 201,
#endif
    ANNOTATED_REGIONS                    = 202,
#if SHUTTER_INTERVAL_SEI_MESSAGE
    SHUTTER_INTERVAL_INFO                = 203,
#endif
  };

  SEI() {}
  virtual ~SEI() {}

  static const TChar *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;

  static const std::vector <SEI::PayloadType> prefix_sei_messages;
  static const std::vector <SEI::PayloadType> suffix_sei_messages;
  static const std::vector <SEI::PayloadType> regional_nesting_sei_messages;
};


typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
Void deleteSEIs (SEIMessages &seiList);


class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target);

  SEIBufferingPeriod()
  : m_bpSeqParameterSetId (0)
  , m_rapCpbParamsPresentFlag (false)
  , m_cpbDelayOffset      (0)
  , m_dpbDelayOffset      (0)
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalDelayOffset, 0, sizeof(m_initialCpbRemovalDelayOffset));
    ::memset(m_initialAltCpbRemovalDelay, 0, sizeof(m_initialAltCpbRemovalDelay));
    ::memset(m_initialAltCpbRemovalDelayOffset, 0, sizeof(m_initialAltCpbRemovalDelayOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  UInt m_bpSeqParameterSetId;
  Bool m_rapCpbParamsPresentFlag;
  UInt m_cpbDelayOffset;
  UInt m_dpbDelayOffset;
  UInt m_initialCpbRemovalDelay         [MAX_CPB_CNT][2];
  UInt m_initialCpbRemovalDelayOffset   [MAX_CPB_CNT][2];
  UInt m_initialAltCpbRemovalDelay      [MAX_CPB_CNT][2];
  UInt m_initialAltCpbRemovalDelayOffset[MAX_CPB_CNT][2];
  Bool m_concatenationFlag;
  UInt m_auCpbRemovalDelayDelta;
};


class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target);

  SEIPictureTiming()
  : m_picStruct               (0)
  , m_sourceScanType          (0)
  , m_duplicateFlag           (false)
  , m_picDpbOutputDuDelay     (0)
  {}
  virtual ~SEIPictureTiming()
  {
  }

  UInt  m_picStruct;
  UInt  m_sourceScanType;
  Bool  m_duplicateFlag;

  UInt  m_auCpbRemovalDelay;
  UInt  m_picDpbOutputDelay;
  UInt  m_picDpbOutputDuDelay;
  UInt  m_numDecodingUnitsMinus1;
  Bool  m_duCommonCpbRemovalDelayFlag;
  UInt  m_duCommonCpbRemovalDelayMinus1;
  std::vector<UInt> m_numNalusInDuMinus1;
  std::vector<UInt> m_duCpbRemovalDelayMinus1;
};


class SEIPanScanRect : public SEI
{
public:
  PayloadType payloadType() const { return PAN_SCAN_RECT; }

  SEIPanScanRect() {}
  virtual ~SEIPanScanRect() {}

  struct PanScanRect
  {
    Int leftOffset;
    Int rightOffset;
    Int topOffset;
    Int bottomOffset;
  };

  UInt m_panScanRectId;
  Bool m_panScanRectCancelFlag;
  std::vector<PanScanRect> m_panScanRectRegions;
  Bool m_panScanRectPersistenceFlag;
};


class SEIFillerPayload : public SEI
{
public:
  PayloadType payloadType() const { return FILLER_PAYLOAD; }

  SEIFillerPayload() {}
  virtual ~SEIFillerPayload() {}

  UInt m_numFillerFFBytes;
};


class SEIUserDataRegistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  virtual ~SEIUserDataRegistered() {}

  UShort m_ituCountryCode;
  std::vector<UChar> m_userData;
};


static const UInt ISO_IEC_11578_LEN=16;

class SEIUserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIUserDataUnregistered() {}
  virtual ~SEIUserDataUnregistered() { }

  UChar m_uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  std::vector<UChar> m_userData;
};


class SEIRecoveryPoint : public SEI
{
public:
  PayloadType payloadType() const { return RECOVERY_POINT; }

  SEIRecoveryPoint() {}
  virtual ~SEIRecoveryPoint() {}

  Int  m_recoveryPocCnt;
  Bool m_exactMatchingFlag;
  Bool m_brokenLinkFlag;
};


class SEISceneInfo : public SEI
{
public:
  PayloadType payloadType() const { return SCENE_INFO; }

  SEISceneInfo() {}
  virtual ~SEISceneInfo() {}

  Bool m_bSceneInfoPresentFlag;
  Bool m_bPrevSceneIdValidFlag;
  UInt m_sceneId;
  UInt m_sceneTransitionType;
  UInt m_secondSceneId;
};


class SEIPictureSnapshot : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_SNAPSHOT; }

  SEIPictureSnapshot() {}
  virtual ~SEIPictureSnapshot() {}

  UInt m_snapshotId;
};


class SEIProgressiveRefinementSegmentStart : public SEI
{
public:
  PayloadType payloadType() const { return PROGRESSIVE_REFINEMENT_SEGMENT_START; }

  SEIProgressiveRefinementSegmentStart() {}
  virtual ~SEIProgressiveRefinementSegmentStart() {}

  UInt m_progressiveRefinementId;
  UInt m_picOrderCntDelta;
};


class SEIProgressiveRefinementSegmentEnd: public SEI
{
public:
  PayloadType payloadType() const { return PROGRESSIVE_REFINEMENT_SEGMENT_END; }

  SEIProgressiveRefinementSegmentEnd() {}
  virtual ~SEIProgressiveRefinementSegmentEnd() {}

  UInt m_progressiveRefinementId;
};


class SEIFilmGrainCharacteristics: public SEI
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  virtual ~SEIFilmGrainCharacteristics() {}

  Bool      m_filmGrainCharacteristicsCancelFlag;
  UChar     m_filmGrainModelId;
  Bool      m_separateColourDescriptionPresentFlag;
  UChar     m_filmGrainBitDepthLumaMinus8;
  UChar     m_filmGrainBitDepthChromaMinus8;
  Bool      m_filmGrainFullRangeFlag;
  UChar     m_filmGrainColourPrimaries;
  UChar     m_filmGrainTransferCharacteristics;
  UChar     m_filmGrainMatrixCoeffs;
  UChar     m_blendingModeId;
  UChar     m_log2ScaleFactor;

  struct CompModelIntensityValues
  {
    UChar intensityIntervalLowerBound;
    UChar intensityIntervalUpperBound;
    std::vector<Int> compModelValue;
  };

  struct CompModel
  {
    Bool  bPresentFlag;
    UChar numModelValues; // this must be the same as intensityValues[*].compModelValue.size()
#if JVET_X0048_X0103_FILM_GRAIN
    UInt  numIntensityIntervals;
#endif
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel m_compModel[MAX_NUM_COMPONENT];
  Bool      m_filmGrainCharacteristicsPersistenceFlag;
};


class SEIPostFilterHint: public SEI
{
public:
  PayloadType payloadType() const { return POST_FILTER_HINT; }

  SEIPostFilterHint() {}
  virtual ~SEIPostFilterHint() {}

  UInt             m_filterHintSizeY;
  UInt             m_filterHintSizeX;
  UInt             m_filterHintType;
  Bool             m_bIsMonochrome;
  std::vector<Int> m_filterHintValues; // values stored in linear array, [ ( ( component * sizeY + y ) * SizeX ) + x ]
};


class SEIToneMappingInfo : public SEI
{
public:
  PayloadType payloadType() const { return TONE_MAPPING_INFO; }
  SEIToneMappingInfo() {}
  virtual ~SEIToneMappingInfo() {}

  Int    m_toneMapId;
  Bool   m_toneMapCancelFlag;
  Bool   m_toneMapPersistenceFlag;
  Int    m_codedDataBitDepth;
  Int    m_targetBitDepth;
  Int    m_modelId;
  Int    m_minValue;
  Int    m_maxValue;
  Int    m_sigmoidMidpoint;
  Int    m_sigmoidWidth;
  std::vector<Int> m_startOfCodedInterval;
  Int    m_numPivots;
  std::vector<Int> m_codedPivotValue;
  std::vector<Int> m_targetPivotValue;
  Int    m_cameraIsoSpeedIdc;
  Int    m_cameraIsoSpeedValue;
  Int    m_exposureIndexIdc;
  Int    m_exposureIndexValue;
  Bool   m_exposureCompensationValueSignFlag;
  Int    m_exposureCompensationValueNumerator;
  Int    m_exposureCompensationValueDenomIdc;
  Int    m_refScreenLuminanceWhite;
  Int    m_extendedRangeWhiteLevel;
  Int    m_nominalBlackLevelLumaCodeValue;
  Int    m_nominalWhiteLevelLumaCodeValue;
  Int    m_extendedWhiteLevelLumaCodeValue;
};


class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  Int  m_arrangementId;
  Bool m_arrangementCancelFlag;
  Int  m_arrangementType;
  Bool m_quincunxSamplingFlag;
  Int  m_contentInterpretationType;
  Bool m_spatialFlippingFlag;
  Bool m_frame0FlippedFlag;
  Bool m_fieldViewsFlag;
  Bool m_currentFrameIsFrame0Flag;
  Bool m_frame0SelfContainedFlag;
  Bool m_frame1SelfContainedFlag;
  Int  m_frame0GridPositionX;
  Int  m_frame0GridPositionY;
  Int  m_frame1GridPositionX;
  Int  m_frame1GridPositionY;
  Int  m_arrangementReservedByte;
  Bool m_arrangementPersistenceFlag;
  Bool m_upsampledAspectRatio;
};


class SEIDisplayOrientation : public SEI
{
public:
  PayloadType payloadType() const { return DISPLAY_ORIENTATION; }

  SEIDisplayOrientation()
    : cancelFlag(true)
    , persistenceFlag(0)
    , extensionFlag(false)
    {}
  virtual ~SEIDisplayOrientation() {}

  Bool cancelFlag;
  Bool horFlip;
  Bool verFlip;

  UInt anticlockwiseRotation;
  Bool persistenceFlag;
  Bool extensionFlag;
};


class SEIGreenMetadataInfo : public SEI
{
public:
    PayloadType payloadType() const { return GREEN_METADATA; }
    SEIGreenMetadataInfo() {}

    virtual ~SEIGreenMetadataInfo() {}

    UInt m_greenMetadataType;
    UInt m_xsdMetricType;
    UInt m_xsdMetricValue;
};


class SEISOPDescription : public SEI
{
public:
  PayloadType payloadType() const { return SOP_DESCRIPTION; }

