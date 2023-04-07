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
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      UInt reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    UInt dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);
}


Void SEIReader::xParseSEIBufferingPeriod(SEIBufferingPeriod& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  Int i, nalOrVcl;
  UInt code;

  const TComVUI *pVUI = sps->getVuiParameters();
  const TComHRD *pHRD = pVUI->getHrdParameters();

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_seq_parameter_set_id" );                         sei.m_bpSeqParameterSetId     = code;
  if( !pHRD->getSubPicCpbParamsPresentFlag() )
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "irap_cpb_params_present_flag" );                   sei.m_rapCpbParamsPresentFlag = code;
  }
  if( sei.m_rapCpbParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, pHRD->getCpbRemovalDelayLengthMinus1() + 1, code, "cpb_delay_offset" );      sei.m_cpbDelayOffset = code;
    sei_read_code( pDecodedMessageOutputStream, pHRD->getDpbOutputDelayLengthMinus1()  + 1, code, "dpb_delay_offset" );      sei.m_dpbDelayOffset = code;
  }

  //read splicing flag and cpb_removal_delay_delta
  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei.m_concatenationFlag = code;
  sei_read_code( pDecodedMessageOutputStream, ( pHRD->getCpbRemovalDelayLengthMinus1() + 1 ), code, "au_cpb_removal_delay_delta_minus1" );
  sei.m_auCpbRemovalDelayDelta = code + 1;

  for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
  {
    if( ( ( nalOrVcl == 0 ) && ( pHRD->getNalHrdParametersPresentFlag() ) ) ||
        ( ( nalOrVcl == 1 ) && ( pHRD->getVclHrdParametersPresentFlag() ) ) )
    {
      for( i = 0; i < ( pHRD->getCpbCntMinus1( 0 ) + 1 ); i ++ )
      {
        sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_cpb_removal_delay":"nal_initial_cpb_removal_delay" );
        sei.m_initialCpbRemovalDelay[i][nalOrVcl] = code;
        sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_cpb_removal_offset":"nal_initial_cpb_removal_offset" );
        sei.m_initialCpbRemovalDelayOffset[i][nalOrVcl] = code;
        if( pHRD->getSubPicCpbParamsPresentFlag() || sei.m_rapCpbParamsPresentFlag )
        {
          sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_alt_cpb_removal_delay":"nal_initial_alt_cpb_removal_delay" );
          sei.m_initialAltCpbRemovalDelay[i][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_alt_cpb_removal_offset":"nal_initial_alt_cpb_removal_offset" );
          sei.m_initialAltCpbRemovalDelayOffset[i][nalOrVcl] = code;
        }
      }
    }
  }
}


Void SEIReader::xParseSEIPictureTiming(SEIPictureTiming& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt code;

  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  if( vui->getFrameFieldInfoPresentFlag() )
  {
    sei_read_code( pDecodedMessageOutputStream, 4, code, "pic_struct" );             sei.m_picStruct            = code;
    sei_read_code( pDecodedMessageOutputStream, 2, code, "source_scan_type" );       sei.m_sourceScanType       = code;
    sei_read_flag( pDecodedMessageOutputStream,    code, "duplicate_flag" );         sei.m_duplicateFlag        = (code == 1);
  }

  if( hrd->getCpbDpbDelaysPresentFlag())
  {
    sei_read_code( pDecodedMessageOutputStream, ( hrd->getCpbRemovalDelayLengthMinus1() + 1 ), code, "au_cpb_removal_delay_minus1" );
    sei.m_auCpbRemovalDelay = code + 1;
    sei_read_code( pDecodedMessageOutputStream, ( hrd->getDpbOutputDelayLengthMinus1() + 1 ), code, "pic_dpb_output_delay" );
    sei.m_picDpbOutputDelay = code;

    if(hrd->getSubPicCpbParamsPresentFlag())
    {
      sei_read_code( pDecodedMessageOutputStream, hrd->getDpbOutputDelayDuLengthMinus1()+1, code, "pic_dpb_output_du_delay" );
      sei.m_picDpbOutputDuDelay = code;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() && hrd->getSubPicCpbParamsInPicTimingSEIFlag() )
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "num_decoding_units_minus1");
      sei.m_numDecodingUnitsMinus1 = code;
      sei_read_flag( pDecodedMessageOutputStream, code, "du_common_cpb_removal_delay_flag" );
      sei.m_duCommonCpbRemovalDelayFlag = code;
      if( sei.m_duCommonCpbRemovalDelayFlag )
      {
        sei_read_code( pDecodedMessageOutputStream, ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ), code, "du_common_cpb_removal_delay_increment_minus1" );
        sei.m_duCommonCpbRemovalDelayMinus1 = code;
      }
      sei.m_numNalusInDuMinus1.resize(sei.m_numDecodingUnitsMinus1 + 1 );
      sei.m_duCpbRemovalDelayMinus1.resize( sei.m_numDecodingUnitsMinus1 + 1 );

      for( i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
      {
        sei_read_uvlc( pDecodedMessageOutputStream, code, "num_nalus_in_du_minus1[i]");
        sei.m_numNalusInDuMinus1[ i ] = code;
        if( ( !sei.m_duCommonCpbRemovalDelayFlag ) && ( i < sei.m_numDecodingUnitsMinus1 ) )
        {
          sei_read_code( pDecodedMessageOutputStream, ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ), code, "du_cpb_removal_delay_minus1[i]" );
          sei.m_duCpbRemovalDelayMinus1[ i ] = code;
        }
      }
    }
  }
}


Void SEIReader::xParseSEIPanScanRect(SEIPanScanRect& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, code, "pan_scan_rect_id" );          sei.m_panScanRectId = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "pan_scan_rect_cancel_flag" ); sei.m_panScanRectCancelFlag = code!=0;
  if (!sei.m_panScanRectCancelFlag)
  {
    UInt numRegions;
    sei_read_uvlc( pDecodedMessageOutputStream, numRegions, "pan_scan_cnt_minus1" ); numRegions++;
    sei.m_panScanRectRegions.resize(numRegions);
    for(UInt region=0; region<numRegions; region++)
    {
      SEIPanScanRect::PanScanRect &rect=sei.m_panScanRectRegions[region];
      Int  i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_left_offset[i]" );   rect.leftOffset   = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_right_offset[i]" );  rect.rightOffset  = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_top_offset[i]" );    rect.topOffset    = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_bottom_offset[i]" ); rect.bottomOffset = i;
    }
    sei_read_flag( pDecodedMessageOutputStream, code, "pan_scan_rect_persistence_flag" ); sei.m_panScanRectPersistenceFlag = code!=0;
  }
  else
  {
    sei.m_panScanRectRegions.clear();
    sei.m_panScanRectPersistenceFlag=false;
  }
}


Void SEIReader::xParseSEIFillerPayload(SEIFillerPayload& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei.m_numFillerFFBytes = payloadSize;
  Bool allBytesWereFF=true;
  for(UInt k=0; k<payloadSize; k++)
  {
    UInt code;
    sei_read_code( NULL, 8, code, "ff_byte" );
    if (code!=0xff) allBytesWereFF=false;
  }
  if (pDecodedMessageOutputStream && !allBytesWereFF)
  {
    (*pDecodedMessageOutputStream) << "  not all filler payload bytes were 0xff\n";
  }
}


Void SEIReader::xParseSEIUserDataRegistered(SEIUserDataRegistered& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  UInt code;
  assert(payloadSize>0);
  sei_read_code( pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code" ); payloadSize--;
  if (code == 255)
  {
    assert(payloadSize>0);
    sei_read_code( pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte" ); payloadSize--;
    code+=255;
  }
  sei.m_ituCountryCode = code;
  sei.m_userData.resize(payloadSize);
  for (UInt i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code( NULL, 8, code, "itu_t_t35_payload_byte" );
    sei.m_userData[i] = code;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << sei.m_userData.size() << "\n";
  }
}


Void SEIReader::xParseSEIUserDataUnregistered(SEIUserDataUnregistered &sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  assert(payloadSize >= ISO_IEC_11578_LEN);
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (UInt i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578");
    sei.m_uuid_iso_iec_11578[i] = val;
  }

  sei.m_userData.resize(payloadSize - ISO_IEC_11578_LEN);
  for (UInt i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code( NULL, 8, val, "user_data_payload_byte" );
    sei.m_userData[i] = val;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << sei.m_userData.size() << "\n";
  }
}


Void SEIReader::xParseSEIRecoveryPoint(SEIRecoveryPoint& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int  iCode;
  UInt uiCode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_svlc( pDecodedMessageOutputStream, iCode,  "recovery_poc_cnt" );      sei.m_recoveryPocCnt     = iCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode, "exact_matching_flag" );   sei.m_exactMatchingFlag  = uiCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode, "broken_link_flag" );      sei.m_brokenLinkFlag     = uiCode;
}


Void SEIReader::xParseSEISceneInfo(SEISceneInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, code, "scene_info_present_flag" ); sei.m_bSceneInfoPresentFlag = code!=0;
  if (sei.m_bSceneInfoPresentFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "prev_scene_id_valid_flag" ); sei.m_bPrevSceneIdValidFlag = code!=0;
    sei_read_uvlc( pDecodedMessageOutputStream, code, "scene_id" );                 sei.m_sceneId = code;
    sei_read_uvlc( pDecodedMessageOutputStream, code, "scene_transition_type" );    sei.m_sceneTransitionType = code;
    if (sei.m_sceneTransitionType > 3)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "second_scene_id" );        sei.m_secondSceneId = code;
    }
    else
    {
      sei.m_secondSceneId = 0; // set to known value.
    }
  }
}


Void SEIReader::xParseSEIPictureSnapshot(SEIPictureSnapshot& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "snapshot_id" ); sei.m_snapshotId = code;
}


Void SEIReader::xParseSEIProgressiveRefinementSegmentStart(SEIProgressiveRefinementSegmentStart& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "progressive_refinement_id" ); sei.m_progressiveRefinementId = code;
  sei_read_uvlc( pDecodedMessageOutputStream, code, "pic_order_cnt_delta" );       sei.m_picOrderCntDelta = code;
}


Void SEIReader::xParseSEIProgressiveRefinementSegmentEnd(SEIProgressiveRefinementSegmentEnd& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "progressive_refinement_id" ); sei.m_progressiveRefinementId = code;
}


Void SEIReader::xParseSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

