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

/**
 \file     SEIread.cpp
 \brief    reading functionality for SEI messages
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/TComSlice.h"
#include "SyntaxElementParser.h"
#include "SEIread.h"
#include "TLibCommon/TComPicYuv.h"
#include <iomanip>


//! \ingroup TLibDecoder
//! \{


#if ENC_DEC_TRACE
Void  xTraceSEIHeader()
{
  fprintf( g_hTrace, "=========== SEI message ===========\n");
}

Void  xTraceSEIMessageType(SEI::PayloadType payloadType)
{
  fprintf( g_hTrace, "=========== %s SEI message ===========\n", SEI::getSEIMessageString(payloadType));
}
#endif

Void SEIReader::sei_read_scode(std::ostream *pOS, UInt uiLength, Int& ruiCode, const TChar *pSymbolName)
{
  READ_SCODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_code(std::ostream *pOS, UInt uiLength, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_CODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_uvlc(std::ostream *pOS, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_UVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_svlc(std::ostream *pOS, Int& ruiCode, const TChar *pSymbolName)
{
  READ_SVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_flag(std::ostream *pOS, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_FLAG(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

static inline Void output_sei_message_header(SEI &sei, std::ostream *pDecodedMessageOutputStream, UInt payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI::getSEIMessageString(sei.payloadType())); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw((int)seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
  }
}

#undef READ_SCODE
#undef READ_CODE
#undef READ_SVLC
#undef READ_UVLC
#undef READ_FLAG


/**
 * unmarshal a single SEI message from bitstream bs
 */
Void SEIReader::parseSEImessage(TComInputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  setBitstream(bs);

  assert(!m_pcBitstream->getNumBitsUntilByteAligned());
  do
  {
    if(nalUnitType == NAL_UNIT_PREFIX_SEI)
    {
      xReadSEImessage(seis, nalUnitType, sps, pDecodedMessageOutputStream, SEI::prefix_sei_messages, std::string("prefix SEI"));
    }
    else if (nalUnitType == NAL_UNIT_SUFFIX_SEI)
    {
      xReadSEImessage(seis, nalUnitType, sps, pDecodedMessageOutputStream, SEI::suffix_sei_messages, std::string("suffix SEI"));
    }
    else
    {
      std::cerr << "Unsupported SEI NAL unit type '" << nalUnitType << "'" << std::endl;
      exit(EXIT_FAILURE);
    }

    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    assert(!m_pcBitstream->getNumBitsUntilByteAligned());
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  xReadRbspTrailingBits();
}
Void SEIReader::xReadSEIPayloadData(Int const payloadType, Int const payloadSize, SEI *&sei, const NalUnitType nalUnitType, const TComSPS *sps, 
  std::ostream *pDecodedMessageOutputStream, std::string const &typeName)
{
  switch(payloadType)
  {
    case SEI::BUFFERING_PERIOD:
      if (!sps)
      {
        printf ("Warning: Found Buffering period SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIBufferingPeriod;
        xParseSEIBufferingPeriod((SEIBufferingPeriod&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::PICTURE_TIMING:
      if (!sps)
      {
        printf ("Warning: Found Picture timing SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIPictureTiming;
        xParseSEIPictureTiming((SEIPictureTiming&)*sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::PAN_SCAN_RECT:
      sei = new SEIPanScanRect;
      xParseSEIPanScanRect((SEIPanScanRect&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILLER_PAYLOAD:
      sei = new SEIFillerPayload;
      xParseSEIFillerPayload((SEIFillerPayload&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:
      sei = new SEIUserDataRegistered;
      xParseSEIUserDataRegistered((SEIUserDataRegistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_UNREGISTERED:
      sei = new SEIUserDataUnregistered;
      xParseSEIUserDataUnregistered((SEIUserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::RECOVERY_POINT:
      sei = new SEIRecoveryPoint;
      xParseSEIRecoveryPoint((SEIRecoveryPoint&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SCENE_INFO:
      sei = new SEISceneInfo;
      xParseSEISceneInfo((SEISceneInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PICTURE_SNAPSHOT:
      sei = new SEIPictureSnapshot;
      xParseSEIPictureSnapshot((SEIPictureSnapshot&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START:
      sei = new SEIProgressiveRefinementSegmentStart;
      xParseSEIProgressiveRefinementSegmentStart((SEIProgressiveRefinementSegmentStart&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:
      sei = new SEIProgressiveRefinementSegmentEnd;
      xParseSEIProgressiveRefinementSegmentEnd((SEIProgressiveRefinementSegmentEnd&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILM_GRAIN_CHARACTERISTICS:
      sei = new SEIFilmGrainCharacteristics;
      xParseSEIFilmGrainCharacteristics((SEIFilmGrainCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::POST_FILTER_HINT:
      if (!sps)
      {
        printf ("Warning: post filter hint SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIPostFilterHint;
        xParseSEIPostFilterHint((SEIPostFilterHint&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::TONE_MAPPING_INFO:
      sei = new SEIToneMappingInfo;
      xParseSEIToneMappingInfo((SEIToneMappingInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FRAME_PACKING:
      sei = new SEIFramePacking;
      xParseSEIFramePacking((SEIFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DISPLAY_ORIENTATION:
      sei = new SEIDisplayOrientation;
      xParseSEIDisplayOrientation((SEIDisplayOrientation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::GREEN_METADATA:
      sei = new SEIGreenMetadataInfo;
      xParseSEIGreenMetadataInfo((SEIGreenMetadataInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SOP_DESCRIPTION:
      sei = new SEISOPDescription;
      xParseSEISOPDescription((SEISOPDescription&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODED_PICTURE_HASH:
      sei = new SEIDecodedPictureHash;
      xParseSEIDecodedPictureHash((SEIDecodedPictureHash&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::ACTIVE_PARAMETER_SETS:
      sei = new SEIActiveParameterSets;
      xParseSEIActiveParameterSets((SEIActiveParameterSets&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODING_UNIT_INFO:
      if (!sps)
      {
        printf ("Warning: Found Decoding unit SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIDecodingUnitInfo;
        xParseSEIDecodingUnitInfo((SEIDecodingUnitInfo&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::TEMPORAL_LEVEL0_INDEX:
      sei = new SEITemporalLevel0Index;
      xParseSEITemporalLevel0Index((SEITemporalLevel0Index&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SCALABLE_NESTING:
      sei = new SEIScalableNesting;
      xParseSEIScalableNesting((SEIScalableNesting&) *sei, nalUnitType, payloadSize, sps, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_REFRESH_INFO:
      sei = new SEIRegionRefreshInfo;
      xParseSEIRegionRefreshInfo((SEIRegionRefreshInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::NO_DISPLAY:
      sei = new SEINoDisplay;
      xParseSEINoDisplay((SEINoDisplay&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::TIME_CODE:
      sei = new SEITimeCode;
      xParseSEITimeCode((SEITimeCode&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
      sei = new SEIMasteringDisplayColourVolume;
      xParseSEIMasteringDisplayColourVolume((SEIMasteringDisplayColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SEGM_RECT_FRAME_PACKING:
      sei = new SEISegmentedRectFramePacking;
      xParseSEISegmentedRectFramePacking((SEISegmentedRectFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:
      sei = new SEITempMotionConstrainedTileSets;
      xParseSEITempMotionConstraintsTileSets((SEITempMotionConstrainedTileSets&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#if MCTS_EXTRACTION
    case SEI::MCTS_EXTRACTION_INFO_SET:
      sei = new SEIMCTSExtractionInfoSet;
      xParseSEIMCTSExtractionInfoSet((SEIMCTSExtractionInfoSet&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    case SEI::CHROMA_RESAMPLING_FILTER_HINT:
      sei = new SEIChromaResamplingFilterHint;
      xParseSEIChromaResamplingFilterHint((SEIChromaResamplingFilterHint&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::KNEE_FUNCTION_INFO:
      sei = new SEIKneeFunctionInfo;
      xParseSEIKneeFunctionInfo((SEIKneeFunctionInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::COLOUR_REMAPPING_INFO:
      sei = new SEIColourRemappingInfo;
      xParseSEIColourRemappingInfo((SEIColourRemappingInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DEINTERLACE_FIELD_IDENTIFICATION:
      sei = new SEIDeinterlaceFieldIdentification;
      xParseSEIDeinterlaceFieldIdentification((SEIDeinterlaceFieldIdentification&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_LIGHT_LEVEL_INFO:
      sei = new SEIContentLightLevelInfo;
      xParseSEIContentLightLevelInfo((SEIContentLightLevelInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DEPENDENT_RAP_INDICATION:
      sei = new SEIDependentRAPIndication;
      xParseSEIDependentRAPIndication((SEIDependentRAPIndication&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CODED_REGION_COMPLETION:
      sei = new SEICodedRegionCompletion;
      xParseSEICodedRegionCompletion((SEICodedRegionCompletion&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
      sei = new SEIAlternativeTransferCharacteristics;
      xParseSEIAlternativeTransferCharacteristics((SEIAlternativeTransferCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:
      sei = new SEIAmbientViewingEnvironment;
      xParseSEIAmbientViewingEnvironment((SEIAmbientViewingEnvironment&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_COLOUR_VOLUME:
      sei = new SEIContentColourVolume;
      xParseSEIContentColourVolume((SEIContentColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::EQUIRECTANGULAR_PROJECTION:
      sei = new SEIEquirectangularProjection;
      xParseSEIEquirectangularProjection((SEIEquirectangularProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SPHERE_ROTATION:
      sei = new SEISphereRotation;
      xParseSEISphereRotation((SEISphereRotation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::OMNI_VIEWPORT:
      sei = new SEIOmniViewport;
      xParseSEIOmniViewport((SEIOmniViewport&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CUBEMAP_PROJECTION:
      sei = new SEICubemapProjection;
      xParseSEICubemapProjection((SEICubemapProjection&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_WISE_PACKING:
      sei = new SEIRegionWisePacking;
      xParseSEIRegionWisePacking((SEIRegionWisePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;

    case SEI::ANNOTATED_REGIONS:
      sei = new SEIAnnotatedRegions;
      xParseSEIAnnotatedRegions((SEIAnnotatedRegions&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FISHEYE_VIDEO_INFO:
      sei = new SEIFisheyeVideoInfo;
      xParseSEIFisheyeVideoInfo((SEIFisheyeVideoInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGIONAL_NESTING:
      sei = new SEIRegionalNesting;
      xParseSEIRegionalNesting((SEIRegionalNesting&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      break;
#if SHUTTER_INTERVAL_SEI_MESSAGE
    case SEI::SHUTTER_INTERVAL_INFO:
      sei = new SEIShutterIntervalInfo;
      xParseSEIShutterInterval((SEIShutterIntervalInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JCTVC_AD0021_SEI_MANIFEST
    case SEI::SEI_MANIFEST:
      sei = new SEIManifest;
      xParseSEISEIManifest((SEIManifest&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    case SEI::SEI_PREFIX_INDICATION:
      sei = new SEIPrefixIndication;
      xParseSEISEIPrefixIndication((SEIPrefixIndication&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    default:
      for (UInt i = 0; i < payloadSize; i++)
      {
        UInt seiByte;
        std::string msg = std::string("unknown ")+typeName+std::string(" payload byte");
        sei_read_code (NULL, 8, seiByte, msg.c_str());
      }
      printf ("Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown "<< typeName << " message (payloadType = " << payloadType << ") was found!\n";
      }
      break;
    }
}

Void SEIReader::xReadSEImessage(SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream, const vector<SEI::PayloadType>& allowedSeiTypes, std::string const &typeName)
{
#if ENC_DEC_TRACE
  xTraceSEIHeader();
#endif
  Int payloadType = 0;
  UInt val = 0;

  do
  {
    sei_read_code(NULL, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  UInt payloadSize = 0;
  do
  {
    sei_read_code(NULL, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

#if ENC_DEC_TRACE
  xTraceSEIMessageType((SEI::PayloadType)payloadType);
#endif

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  TComInputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  SEI *sei = NULL;

  if (std::find(allowedSeiTypes.begin(), allowedSeiTypes.begin(), payloadType) !=  allowedSeiTypes.end())
  {
    xReadSEIPayloadData(payloadType, payloadSize, sei, nalUnitType, sps, pDecodedMessageOutputStream, typeName);
  } 
  else
  {
    for (UInt i = 0; i < payloadSize; i++)
    {
      UInt seiByte;
      sei_read_code (NULL, 8, seiByte, "unknown SEI payload byte");
    }
    printf ("Unknown SEI message (payloadType = %d) was found!\n", payloadType);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "Unknown SEI message (payloadType = " << payloadType << ") was found!\n";
    }
  }

  if (sei != NULL)
  {
    seis.push_back(sei);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  Int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      UInt reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    Int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    Int finalPayloadBits = 0;
    for (Int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
