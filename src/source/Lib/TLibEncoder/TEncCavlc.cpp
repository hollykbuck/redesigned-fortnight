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

/** \file     TEncCavlc.cpp
    \brief    CAVLC encoder class
*/

#include "../TLibCommon/CommonDef.h"
#include "TEncCavlc.h"
#include "SEIwrite.h"

//! \ingroup TLibEncoder
//! \{

#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
Void  xTraceVPSHeaderEnc()
#else
Void  xTraceVPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTraceSPSHeaderEnc()
#else
Void  xTraceSPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTracePPSHeaderEnc()
#else
Void  xTracePPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n");
}

#if MCTS_EXTRACTION
Void  xTraceSliceHeaderEnc()
#else
Void  xTraceSliceHeader()
#endif
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#if MCTS_EXTRACTION
Void  xTraceAccessUnitDelimiterEnc()
#else
Void  xTraceAccessUnitDelimiter()
#endif
{
  fprintf( g_hTrace, "=========== Access Unit Delimiter ===========\n");
}

#endif

Void AUDWriter::codeAUD(TComBitIf& bs, const Int pictureType)
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceAccessUnitDelimiterEnc();
#else
  xTraceAccessUnitDelimiter();
#endif
#endif

  assert (pictureType < 3);
  setBitstream(&bs);
  WRITE_CODE(pictureType, 3, "pic_type");
  xWriteRbspTrailingBits();
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncCavlc::TEncCavlc()
{
  m_pcBitIf           = NULL;
}

TEncCavlc::~TEncCavlc()
{
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncCavlc::resetEntropy(const TComSlice* /*pSlice*/)
{
}


Void TEncCavlc::codeShortTermRefPicSet( const TComReferencePictureSet* rps, Bool calledFromSliceHeader, Int idx)
{
#if PRINT_RPS_INFO
  Int lastBits = getNumberOfWrittenBits();
#endif
  if (idx > 0)
  {
  WRITE_FLAG( rps->getInterRPSPrediction(), "inter_ref_pic_set_prediction_flag" ); // inter_RPS_prediction_flag
  }
  if (rps->getInterRPSPrediction())
  {
    Int deltaRPS = rps->getDeltaRPS();
    if(calledFromSliceHeader)
    {
      WRITE_UVLC( rps->getDeltaRIdxMinus1(), "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }

    WRITE_CODE( (deltaRPS >=0 ? 0: 1), 1, "delta_rps_sign" ); //delta_rps_sign
    WRITE_UVLC( abs(deltaRPS) - 1, "abs_delta_rps_minus1"); // absolute delta RPS minus 1

    for(Int j=0; j < rps->getNumRefIdc(); j++)
    {
      Int refIdc = rps->getRefIdc(j);
      WRITE_CODE( (refIdc==1? 1: 0), 1, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      if (refIdc != 1)
      {
        WRITE_CODE( refIdc>>1, 1, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
      }
    }
  }
  else
  {
    WRITE_UVLC( rps->getNumberOfNegativePictures(), "num_negative_pics" );
    WRITE_UVLC( rps->getNumberOfPositivePictures(), "num_positive_pics" );
    Int prev = 0;
    for(Int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      WRITE_UVLC( prev-rps->getDeltaPOC(j)-1, "delta_poc_s0_minus1" );
      prev = rps->getDeltaPOC(j);
      WRITE_FLAG( rps->getUsed(j), "used_by_curr_pic_s0_flag");
    }
    prev = 0;
    for(Int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      WRITE_UVLC( rps->getDeltaPOC(j)-prev-1, "delta_poc_s1_minus1" );
      prev = rps->getDeltaPOC(j);
      WRITE_FLAG( rps->getUsed(j), "used_by_curr_pic_s1_flag" );
    }
  }

#if PRINT_RPS_INFO
  printf("irps=%d (%2d bits) ", rps->getInterRPSPrediction(), getNumberOfWrittenBits() - lastBits);
  rps->printDeltaPOC();
#endif
}


Void TEncCavlc::codePPS( const TComPPS* pcPPS )
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTracePPSHeaderEnc();
#else
  xTracePPSHeader();
#endif
#endif

  WRITE_UVLC( pcPPS->getPPSId(),                             "pps_pic_parameter_set_id" );
  WRITE_UVLC( pcPPS->getSPSId(),                             "pps_seq_parameter_set_id" );
  WRITE_FLAG( pcPPS->getDependentSliceSegmentsEnabledFlag()    ? 1 : 0, "dependent_slice_segments_enabled_flag" );
  WRITE_FLAG( pcPPS->getOutputFlagPresentFlag() ? 1 : 0,     "output_flag_present_flag" );
  WRITE_CODE( pcPPS->getNumExtraSliceHeaderBits(), 3,        "num_extra_slice_header_bits");
  WRITE_FLAG( pcPPS->getSignDataHidingEnabledFlag(),         "sign_data_hiding_enabled_flag" );
  WRITE_FLAG( pcPPS->getCabacInitPresentFlag() ? 1 : 0,   "cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->getNumRefIdxL0DefaultActive()-1,     "num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->getNumRefIdxL1DefaultActive()-1,     "num_ref_idx_l1_default_active_minus1");

  WRITE_SVLC( pcPPS->getPicInitQPMinus26(),                  "init_qp_minus26");
  WRITE_FLAG( pcPPS->getConstrainedIntraPred() ? 1 : 0,      "constrained_intra_pred_flag" );
  WRITE_FLAG( pcPPS->getUseTransformSkip() ? 1 : 0,  "transform_skip_enabled_flag" );
  WRITE_FLAG( pcPPS->getUseDQP() ? 1 : 0, "cu_qp_delta_enabled_flag" );
  if ( pcPPS->getUseDQP() )
  {
    WRITE_UVLC( pcPPS->getMaxCuDQPDepth(), "diff_cu_qp_delta_depth" );
  }

  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cb), "pps_cb_qp_offset" );
  WRITE_SVLC( pcPPS->getQpOffset(COMPONENT_Cr), "pps_cr_qp_offset" );

  WRITE_FLAG( pcPPS->getSliceChromaQpFlag() ? 1 : 0,          "pps_slice_chroma_qp_offsets_present_flag" );

  WRITE_FLAG( pcPPS->getUseWP() ? 1 : 0,  "weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcPPS->getWPBiPred() ? 1 : 0, "weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
  WRITE_FLAG( pcPPS->getTransquantBypassEnabledFlag()  ? 1 : 0, "transquant_bypass_enabled_flag" );
  WRITE_FLAG( pcPPS->getTilesEnabledFlag()             ? 1 : 0, "tiles_enabled_flag" );
  WRITE_FLAG( pcPPS->getEntropyCodingSyncEnabledFlag() ? 1 : 0, "entropy_coding_sync_enabled_flag" );
  if( pcPPS->getTilesEnabledFlag() )
  {
    WRITE_UVLC( pcPPS->getNumTileColumnsMinus1(),                                    "num_tile_columns_minus1" );
    WRITE_UVLC( pcPPS->getNumTileRowsMinus1(),                                       "num_tile_rows_minus1" );
    WRITE_FLAG( pcPPS->getTileUniformSpacingFlag(),                                  "uniform_spacing_flag" );
    if( !pcPPS->getTileUniformSpacingFlag() )
    {
      for(UInt i=0; i<pcPPS->getNumTileColumnsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileColumnWidth(i)-1,                                  "column_width_minus1" );
      }
      for(UInt i=0; i<pcPPS->getNumTileRowsMinus1(); i++)
      {
        WRITE_UVLC( pcPPS->getTileRowHeight(i)-1,                                    "row_height_minus1" );
      }
    }
    assert ((pcPPS->getNumTileColumnsMinus1() + pcPPS->getNumTileRowsMinus1()) != 0);
    WRITE_FLAG( pcPPS->getLoopFilterAcrossTilesEnabledFlag()?1 : 0,       "loop_filter_across_tiles_enabled_flag");
  }
  WRITE_FLAG( pcPPS->getLoopFilterAcrossSlicesEnabledFlag()?1 : 0,        "pps_loop_filter_across_slices_enabled_flag");
  WRITE_FLAG( pcPPS->getDeblockingFilterControlPresentFlag()?1 : 0,       "deblocking_filter_control_present_flag");
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    WRITE_FLAG( pcPPS->getDeblockingFilterOverrideEnabledFlag() ? 1 : 0,  "deblocking_filter_override_enabled_flag" );
    WRITE_FLAG( pcPPS->getPPSDeblockingFilterDisabledFlag() ? 1 : 0,      "pps_deblocking_filter_disabled_flag" );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      WRITE_SVLC( pcPPS->getDeblockingFilterBetaOffsetDiv2(),             "pps_beta_offset_div2" );
      WRITE_SVLC( pcPPS->getDeblockingFilterTcOffsetDiv2(),               "pps_tc_offset_div2" );
    }
  }
  WRITE_FLAG( pcPPS->getScalingListPresentFlag() ? 1 : 0,                          "pps_scaling_list_data_present_flag" );
  if( pcPPS->getScalingListPresentFlag() )
  {
    codeScalingList( pcPPS->getScalingList() );
  }
  WRITE_FLAG( pcPPS->getListsModificationPresentFlag(), "lists_modification_present_flag");
  WRITE_UVLC( pcPPS->getLog2ParallelMergeLevelMinus2(), "log2_parallel_merge_level_minus2");
  WRITE_FLAG( pcPPS->getSliceHeaderExtensionPresentFlag() ? 1 : 0, "slice_segment_header_extension_present_flag");

  Bool pps_extension_present_flag=false;
  Bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS]={false};

  pps_extension_flags[PPS_EXT__REXT] = pcPPS->getPpsRangeExtension().settingsDifferFromDefaults(pcPPS->getUseTransformSkip());

  // Other PPS extension flags checked here.

  for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
  {
    pps_extension_present_flag|=pps_extension_flags[i];
  }

  WRITE_FLAG( (pps_extension_present_flag?1:0), "pps_extension_present_flag" );

  if (pps_extension_present_flag)
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const TChar *syntaxStrings[]={ "pps_range_extension_flag",
                                          "pps_multilayer_extension_flag",
                                          "pps_extension_6bits[0]",
                                          "pps_extension_6bits[1]",
                                          "pps_extension_6bits[2]",
                                          "pps_extension_6bits[3]",
                                          "pps_extension_6bits[4]",
                                          "pps_extension_6bits[5]" };
#endif

    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( pps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
          case PPS_EXT__REXT:
            {
              const TComPPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
              if (pcPPS->getUseTransformSkip())
              {
                WRITE_UVLC( ppsRangeExtension.getLog2MaxTransformSkipBlockSize()-2,            "log2_max_transform_skip_block_size_minus2");
              }

              WRITE_FLAG((ppsRangeExtension.getCrossComponentPredictionEnabledFlag() ? 1 : 0), "cross_component_prediction_enabled_flag" );

              WRITE_FLAG(UInt(ppsRangeExtension.getChromaQpOffsetListEnabledFlag()),           "chroma_qp_offset_list_enabled_flag" );
              if (ppsRangeExtension.getChromaQpOffsetListEnabledFlag())
              {
                WRITE_UVLC(ppsRangeExtension.getDiffCuChromaQpOffsetDepth(),                   "diff_cu_chroma_qp_offset_depth");
                WRITE_UVLC(ppsRangeExtension.getChromaQpOffsetListLen() - 1,                   "chroma_qp_offset_list_len_minus1");
                /* skip zero index */
                for (Int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < ppsRangeExtension.getChromaQpOffsetListLen(); cuChromaQpOffsetIdx++)
                {
                  WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "cb_qp_offset_list[i]");
                  WRITE_SVLC(ppsRangeExtension.getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "cr_qp_offset_list[i]");
                }
              }

              WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA),           "log2_sao_offset_scale_luma"   );
              WRITE_UVLC( ppsRangeExtension.getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA),         "log2_sao_offset_scale_chroma" );
            }
            break;
          default:
            assert(pps_extension_flags[i]==false); // Should never get here with an active PPS extension flag.
            break;
        } // switch
      } // if flag present
    } // loop over PPS flags
  } // pps_extension_present_flag is non-zero
  xWriteRbspTrailingBits();
}

Void TEncCavlc::codeVUI( const TComVUI *pcVUI, const TComSPS* pcSPS )
{
#if ENC_DEC_TRACE
  fprintf( g_hTrace, "----------- vui_parameters -----------\n");
#endif
  WRITE_FLAG(pcVUI->getAspectRatioInfoPresentFlag(),            "aspect_ratio_info_present_flag");
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    WRITE_CODE(pcVUI->getAspectRatioIdc(), 8,                   "aspect_ratio_idc" );
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      WRITE_CODE(pcVUI->getSarWidth(), 16,                      "sar_width");
      WRITE_CODE(pcVUI->getSarHeight(), 16,                     "sar_height");
    }
  }
  WRITE_FLAG(pcVUI->getOverscanInfoPresentFlag(),               "overscan_info_present_flag");
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    WRITE_FLAG(pcVUI->getOverscanAppropriateFlag(),             "overscan_appropriate_flag");
  }
  WRITE_FLAG(pcVUI->getVideoSignalTypePresentFlag(),            "video_signal_type_present_flag");
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    WRITE_CODE(pcVUI->getVideoFormat(), 3,                      "video_format");
    WRITE_FLAG(pcVUI->getVideoFullRangeFlag(),                  "video_full_range_flag");
    WRITE_FLAG(pcVUI->getColourDescriptionPresentFlag(),        "colour_description_present_flag");
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      WRITE_CODE(pcVUI->getColourPrimaries(), 8,                "colour_primaries");
      WRITE_CODE(pcVUI->getTransferCharacteristics(), 8,        "transfer_characteristics");
      WRITE_CODE(pcVUI->getMatrixCoefficients(), 8,             "matrix_coeffs");
    }
  }

  WRITE_FLAG(pcVUI->getChromaLocInfoPresentFlag(),              "chroma_loc_info_present_flag");
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    WRITE_UVLC(pcVUI->getChromaSampleLocTypeTopField(),         "chroma_sample_loc_type_top_field");
    WRITE_UVLC(pcVUI->getChromaSampleLocTypeBottomField(),      "chroma_sample_loc_type_bottom_field");
  }

  WRITE_FLAG(pcVUI->getNeutralChromaIndicationFlag(),           "neutral_chroma_indication_flag");
  WRITE_FLAG(pcVUI->getFieldSeqFlag(),                          "field_seq_flag");
  WRITE_FLAG(pcVUI->getFrameFieldInfoPresentFlag(),             "frame_field_info_present_flag");

  Window defaultDisplayWindow = pcVUI->getDefaultDisplayWindow();
  WRITE_FLAG(defaultDisplayWindow.getWindowEnabledFlag(),       "default_display_window_flag");
  if( defaultDisplayWindow.getWindowEnabledFlag() )
  {
    WRITE_UVLC(defaultDisplayWindow.getWindowLeftOffset()  / TComSPS::getWinUnitX(pcSPS->getChromaFormatIdc()), "def_disp_win_left_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowRightOffset() / TComSPS::getWinUnitX(pcSPS->getChromaFormatIdc()), "def_disp_win_right_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowTopOffset()   / TComSPS::getWinUnitY(pcSPS->getChromaFormatIdc()), "def_disp_win_top_offset");
    WRITE_UVLC(defaultDisplayWindow.getWindowBottomOffset()/ TComSPS::getWinUnitY(pcSPS->getChromaFormatIdc()), "def_disp_win_bottom_offset");
  }
  const TimingInfo *timingInfo = pcVUI->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "vui_timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "vui_num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "vui_time_scale");
    WRITE_FLAG(timingInfo->getPocProportionalToTimingFlag(),  "vui_poc_proportional_to_timing_flag");
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      WRITE_UVLC(timingInfo->getNumTicksPocDiffOneMinus1(),   "vui_num_ticks_poc_diff_one_minus1");
    }
    WRITE_FLAG(pcVUI->getHrdParametersPresentFlag(),              "vui_hrd_parameters_present_flag");
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      codeHrdParameters(pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  WRITE_FLAG(pcVUI->getBitstreamRestrictionFlag(),              "bitstream_restriction_flag");
  if (pcVUI->getBitstreamRestrictionFlag())
  {
    WRITE_FLAG(pcVUI->getTilesFixedStructureFlag(),             "tiles_fixed_structure_flag");
    WRITE_FLAG(pcVUI->getMotionVectorsOverPicBoundariesFlag(),  "motion_vectors_over_pic_boundaries_flag");
    WRITE_FLAG(pcVUI->getRestrictedRefPicListsFlag(),           "restricted_ref_pic_lists_flag");
    WRITE_UVLC(pcVUI->getMinSpatialSegmentationIdc(),           "min_spatial_segmentation_idc");
    WRITE_UVLC(pcVUI->getMaxBytesPerPicDenom(),                 "max_bytes_per_pic_denom");
    WRITE_UVLC(pcVUI->getMaxBitsPerMinCuDenom(),                "max_bits_per_min_cu_denom");
    WRITE_UVLC(pcVUI->getLog2MaxMvLengthHorizontal(),           "log2_max_mv_length_horizontal");
    WRITE_UVLC(pcVUI->getLog2MaxMvLengthVertical(),             "log2_max_mv_length_vertical");
  }
}

Void TEncCavlc::codeHrdParameters( const TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1 )
{
  if( commonInfPresentFlag )
  {
    WRITE_FLAG( hrd->getNalHrdParametersPresentFlag() ? 1 : 0 ,  "nal_hrd_parameters_present_flag" );
    WRITE_FLAG( hrd->getVclHrdParametersPresentFlag() ? 1 : 0 ,  "vcl_hrd_parameters_present_flag" );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      WRITE_FLAG( hrd->getSubPicCpbParamsPresentFlag() ? 1 : 0,  "sub_pic_hrd_params_present_flag" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getTickDivisorMinus2(), 8,              "tick_divisor_minus2" );
        WRITE_CODE( hrd->getDuCpbRemovalDelayLengthMinus1(), 5,  "du_cpb_removal_delay_increment_length_minus1" );
        WRITE_FLAG( hrd->getSubPicCpbParamsInPicTimingSEIFlag() ? 1 : 0, "sub_pic_cpb_params_in_pic_timing_sei_flag" );
        WRITE_CODE( hrd->getDpbOutputDelayDuLengthMinus1(), 5,   "dpb_output_delay_du_length_minus1"  );
      }
      WRITE_CODE( hrd->getBitRateScale(), 4,                     "bit_rate_scale" );
      WRITE_CODE( hrd->getCpbSizeScale(), 4,                     "cpb_size_scale" );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        WRITE_CODE( hrd->getDuCpbSizeScale(), 4,                "du_cpb_size_scale" );
      }
      WRITE_CODE( hrd->getInitialCpbRemovalDelayLengthMinus1(), 5, "initial_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getCpbRemovalDelayLengthMinus1(),        5, "au_cpb_removal_delay_length_minus1" );
      WRITE_CODE( hrd->getDpbOutputDelayLengthMinus1(),         5, "dpb_output_delay_length_minus1" );
    }
  }
  Int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    WRITE_FLAG( hrd->getFixedPicRateFlag( i ) ? 1 : 0,          "fixed_pic_rate_general_flag");
    Bool fixedPixRateWithinCvsFlag = true;
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      fixedPixRateWithinCvsFlag = hrd->getFixedPicRateWithinCvsFlag( i );
      WRITE_FLAG( hrd->getFixedPicRateWithinCvsFlag( i ) ? 1 : 0, "fixed_pic_rate_within_cvs_flag");
    }
    if( fixedPixRateWithinCvsFlag )
    {
      WRITE_UVLC( hrd->getPicDurationInTcMinus1( i ),           "elemental_duration_in_tc_minus1");
    }
    else
    {
      WRITE_FLAG( hrd->getLowDelayHrdFlag( i ) ? 1 : 0,           "low_delay_hrd_flag");
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      WRITE_UVLC( hrd->getCpbCntMinus1( i ),                      "cpb_cnt_minus1");
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          WRITE_UVLC( hrd->getBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_value_minus1");
          WRITE_UVLC( hrd->getCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_value_minus1");
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            WRITE_UVLC( hrd->getDuCpbSizeValueMinus1( i, j, nalOrVcl ), "cpb_size_du_value_minus1");
            WRITE_UVLC( hrd->getDuBitRateValueMinus1( i, j, nalOrVcl ), "bit_rate_du_value_minus1");
          }
          WRITE_FLAG( hrd->getCbrFlag( i, j, nalOrVcl ) ? 1 : 0, "cbr_flag");
        }
      }
    }
  }
}

Void TEncCavlc::codeSPS( const TComSPS* pcSPS )
{

  const ChromaFormat format                = pcSPS->getChromaFormatIdc();
  const Bool         chromaEnabled         = isChromaEnabled(format);

#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceSPSHeaderEnc();
#else
  xTraceSPSHeader();
#endif
#endif
  WRITE_CODE( pcSPS->getVPSId (),          4,       "sps_video_parameter_set_id" );
  WRITE_CODE( pcSPS->getMaxTLayers() - 1,  3,       "sps_max_sub_layers_minus1" );
  WRITE_FLAG( pcSPS->getTemporalIdNestingFlag() ? 1 : 0, "sps_temporal_id_nesting_flag" );
  codePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  WRITE_UVLC( pcSPS->getSPSId (),                   "sps_seq_parameter_set_id" );
  WRITE_UVLC( Int(pcSPS->getChromaFormatIdc ()),    "chroma_format_idc" );
  if( format == CHROMA_444 )
  {
    WRITE_FLAG( 0,                                  "separate_colour_plane_flag");
  }

  WRITE_UVLC( pcSPS->getPicWidthInLumaSamples (),   "pic_width_in_luma_samples" );
  WRITE_UVLC( pcSPS->getPicHeightInLumaSamples(),   "pic_height_in_luma_samples" );
  Window conf = pcSPS->getConformanceWindow();

  WRITE_FLAG( conf.getWindowEnabledFlag(),          "conformance_window_flag" );
  if (conf.getWindowEnabledFlag())
  {
    WRITE_UVLC( conf.getWindowLeftOffset()   / TComSPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_left_offset" );
    WRITE_UVLC( conf.getWindowRightOffset()  / TComSPS::getWinUnitX(pcSPS->getChromaFormatIdc() ), "conf_win_right_offset" );
    WRITE_UVLC( conf.getWindowTopOffset()    / TComSPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_top_offset" );
    WRITE_UVLC( conf.getWindowBottomOffset() / TComSPS::getWinUnitY(pcSPS->getChromaFormatIdc() ), "conf_win_bottom_offset" );
  }

  WRITE_UVLC( pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8,                      "bit_depth_luma_minus8" );

  WRITE_UVLC( chromaEnabled ? (pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) - 8):0,  "bit_depth_chroma_minus8" );

  WRITE_UVLC( pcSPS->getBitsForPOC()-4,                 "log2_max_pic_order_cnt_lsb_minus4" );

  const Bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,       "sps_sub_layer_ordering_info_present_flag");
  for(UInt i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcSPS->getMaxDecPicBuffering(i) - 1,       "sps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcSPS->getNumReorderPics(i),               "sps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcSPS->getMaxLatencyIncreasePlus1(i),      "sps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }
  assert( pcSPS->getMaxCUWidth() == pcSPS->getMaxCUHeight() );
  WRITE_UVLC( pcSPS->getLog2MinCodingBlockSize() - 3,                                "log2_min_luma_coding_block_size_minus3" );
  WRITE_UVLC( pcSPS->getLog2DiffMaxMinCodingBlockSize(),                             "log2_diff_max_min_luma_coding_block_size" );
  WRITE_UVLC( pcSPS->getQuadtreeTULog2MinSize() - 2,                                 "log2_min_luma_transform_block_size_minus2" );
  WRITE_UVLC( pcSPS->getQuadtreeTULog2MaxSize() - pcSPS->getQuadtreeTULog2MinSize(), "log2_diff_max_min_luma_transform_block_size" );
  WRITE_UVLC( pcSPS->getQuadtreeTUMaxDepthInter() - 1,                               "max_transform_hierarchy_depth_inter" );
  WRITE_UVLC( pcSPS->getQuadtreeTUMaxDepthIntra() - 1,                               "max_transform_hierarchy_depth_intra" );
  WRITE_FLAG( pcSPS->getScalingListFlag() ? 1 : 0,                                   "scaling_list_enabled_flag" );
  if(pcSPS->getScalingListFlag())
  {
    WRITE_FLAG( pcSPS->getScalingListPresentFlag() ? 1 : 0,                          "sps_scaling_list_data_present_flag" );
    if(pcSPS->getScalingListPresentFlag())
    {
      codeScalingList( pcSPS->getScalingList() );
    }
  }
  WRITE_FLAG( pcSPS->getUseAMP() ? 1 : 0,                                            "amp_enabled_flag" );
  WRITE_FLAG( pcSPS->getUseSAO() ? 1 : 0,                                            "sample_adaptive_offset_enabled_flag");

  WRITE_FLAG( pcSPS->getUsePCM() ? 1 : 0,                                            "pcm_enabled_flag");
  if( pcSPS->getUsePCM() )
  {
    WRITE_CODE( pcSPS->getPCMBitDepth(CHANNEL_TYPE_LUMA) - 1, 4,                            "pcm_sample_bit_depth_luma_minus1" );
    WRITE_CODE( chromaEnabled ? (pcSPS->getPCMBitDepth(CHANNEL_TYPE_CHROMA) - 1) : 0, 4,    "pcm_sample_bit_depth_chroma_minus1" );
    WRITE_UVLC( pcSPS->getPCMLog2MinSize() - 3,                                      "log2_min_pcm_luma_coding_block_size_minus3" );
    WRITE_UVLC( pcSPS->getPCMLog2MaxSize() - pcSPS->getPCMLog2MinSize(),             "log2_diff_max_min_pcm_luma_coding_block_size" );
    WRITE_FLAG( pcSPS->getPCMFilterDisableFlag()?1 : 0,                              "pcm_loop_filter_disable_flag");
  }

  assert( pcSPS->getMaxTLayers() > 0 );

  const TComRPSList* rpsList = pcSPS->getRPSList();

  WRITE_UVLC(rpsList->getNumberOfReferencePictureSets(), "num_short_term_ref_pic_sets" );
  for(Int i=0; i < rpsList->getNumberOfReferencePictureSets(); i++)
  {
    const TComReferencePictureSet*rps = rpsList->getReferencePictureSet(i);
    codeShortTermRefPicSet( rps,false, i);
  }
  WRITE_FLAG( pcSPS->getLongTermRefsPresent() ? 1 : 0,         "long_term_ref_pics_present_flag" );
  if (pcSPS->getLongTermRefsPresent())
  {
    WRITE_UVLC(pcSPS->getNumLongTermRefPicSPS(), "num_long_term_ref_pics_sps" );
    for (UInt k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      WRITE_CODE( pcSPS->getLtRefPicPocLsbSps(k), pcSPS->getBitsForPOC(), "lt_ref_pic_poc_lsb_sps");
      WRITE_FLAG( pcSPS->getUsedByCurrPicLtSPSFlag(k), "used_by_curr_pic_lt_sps_flag[i]");
    }
  }
  WRITE_FLAG( pcSPS->getSPSTemporalMVPEnabledFlag()  ? 1 : 0,  "sps_temporal_mvp_enabled_flag" );

  WRITE_FLAG( pcSPS->getUseStrongIntraSmoothing(),             "strong_intra_smoothing_enable_flag" );

  WRITE_FLAG( pcSPS->getVuiParametersPresentFlag(),            "vui_parameters_present_flag" );
  if (pcSPS->getVuiParametersPresentFlag())
  {
      codeVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  Bool sps_extension_present_flag=false;
  Bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS]={false};

  sps_extension_flags[SPS_EXT__REXT] = pcSPS->getSpsRangeExtension().settingsDifferFromDefaults();

  // Other SPS extension flags checked here.

  for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
  {
    sps_extension_present_flag|=sps_extension_flags[i];
  }

  WRITE_FLAG( (sps_extension_present_flag?1:0), "sps_extension_present_flag" );

  if (sps_extension_present_flag)
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const TChar *syntaxStrings[]={ "sps_range_extension_flag",
                                          "sps_multilayer_extension_flag",
                                          "sps_extension_6bits[0]",
                                          "sps_extension_6bits[1]",
                                          "sps_extension_6bits[2]",
                                          "sps_extension_6bits[3]",
                                          "sps_extension_6bits[4]",
                                          "sps_extension_6bits[5]" };
#endif

    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( sps_extension_flags[i]?1:0, syntaxStrings[i] );
    }

    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
          case SPS_EXT__REXT:
          {
            const TComSPSRExt &spsRangeExtension=pcSPS->getSpsRangeExtension();

            WRITE_FLAG( (spsRangeExtension.getTransformSkipRotationEnabledFlag() ? 1 : 0),      "transform_skip_rotation_enabled_flag");
            WRITE_FLAG( (spsRangeExtension.getTransformSkipContextEnabledFlag() ? 1 : 0),       "transform_skip_context_enabled_flag");
            WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT) ? 1 : 0), "implicit_rdpcm_enabled_flag" );
            WRITE_FLAG( (spsRangeExtension.getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT) ? 1 : 0), "explicit_rdpcm_enabled_flag" );
            WRITE_FLAG( (spsRangeExtension.getExtendedPrecisionProcessingFlag() ? 1 : 0),       "extended_precision_processing_flag" );
            WRITE_FLAG( (spsRangeExtension.getIntraSmoothingDisabledFlag() ? 1 : 0),            "intra_smoothing_disabled_flag" );
            WRITE_FLAG( (spsRangeExtension.getHighPrecisionOffsetsEnabledFlag() ? 1 : 0),       "high_precision_offsets_enabled_flag" );
            WRITE_FLAG( (spsRangeExtension.getPersistentRiceAdaptationEnabledFlag() ? 1 : 0),   "persistent_rice_adaptation_enabled_flag" );
            WRITE_FLAG( (spsRangeExtension.getCabacBypassAlignmentEnabledFlag() ? 1 : 0),       "cabac_bypass_alignment_enabled_flag" );
            break;
          }
          default:
            assert(sps_extension_flags[i]==false); // Should never get here with an active SPS extension flag.
            break;
        }
      }
    }
  }
  xWriteRbspTrailingBits();
}

Void TEncCavlc::codeVPS( const TComVPS* pcVPS )
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceVPSHeaderEnc();
#else
  xTraceVPSHeader();
#endif
#endif
  WRITE_CODE( pcVPS->getVPSId(),                    4,        "vps_video_parameter_set_id" );
  WRITE_FLAG(                                       1,        "vps_base_layer_internal_flag" );
  WRITE_FLAG(                                       1,        "vps_base_layer_available_flag" );
  WRITE_CODE( 0,                                    6,        "vps_max_layers_minus1" );
  WRITE_CODE( pcVPS->getMaxTLayers() - 1,           3,        "vps_max_sub_layers_minus1" );
  WRITE_FLAG( pcVPS->getTemporalNestingFlag(),                "vps_temporal_id_nesting_flag" );
  assert (pcVPS->getMaxTLayers()>1||pcVPS->getTemporalNestingFlag());
  WRITE_CODE( 0xffff,                              16,        "vps_reserved_0xffff_16bits" );
  codePTL( pcVPS->getPTL(), true, pcVPS->getMaxTLayers() - 1 );
  const Bool subLayerOrderingInfoPresentFlag = 1;
  WRITE_FLAG(subLayerOrderingInfoPresentFlag,              "vps_sub_layer_ordering_info_present_flag");
  for(UInt i=0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    WRITE_UVLC( pcVPS->getMaxDecPicBuffering(i) - 1,       "vps_max_dec_pic_buffering_minus1[i]" );
    WRITE_UVLC( pcVPS->getNumReorderPics(i),               "vps_max_num_reorder_pics[i]" );
    WRITE_UVLC( pcVPS->getMaxLatencyIncrease(i),           "vps_max_latency_increase_plus1[i]" );
    if (!subLayerOrderingInfoPresentFlag)
    {
      break;
    }
  }

  assert( pcVPS->getNumHrdParameters() <= MAX_VPS_NUM_HRD_PARAMETERS );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  WRITE_CODE( pcVPS->getMaxNuhReservedZeroLayerId(), 6,     "vps_max_layer_id" );
  WRITE_UVLC( pcVPS->getMaxOpSets() - 1,                    "vps_num_layer_sets_minus1" );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      // Only applicable for version 1
      // pcVPS->setLayerIdIncludedFlag( true, opsIdx, i );
      WRITE_FLAG( pcVPS->getLayerIdIncludedFlag( opsIdx, i ) ? 1 : 0, "layer_id_included_flag[opsIdx][i]" );
    }
  }
  const TimingInfo *timingInfo = pcVPS->getTimingInfo();
  WRITE_FLAG(timingInfo->getTimingInfoPresentFlag(),          "vps_timing_info_present_flag");
  if(timingInfo->getTimingInfoPresentFlag())
  {
    WRITE_CODE(timingInfo->getNumUnitsInTick(), 32,           "vps_num_units_in_tick");
    WRITE_CODE(timingInfo->getTimeScale(),      32,           "vps_time_scale");
    WRITE_FLAG(timingInfo->getPocProportionalToTimingFlag(),  "vps_poc_proportional_to_timing_flag");
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      WRITE_UVLC(timingInfo->getNumTicksPocDiffOneMinus1(),   "vps_num_ticks_poc_diff_one_minus1");
    }
    WRITE_UVLC( pcVPS->getNumHrdParameters(),                 "vps_num_hrd_parameters" );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      for( UInt i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
      {
        // Only applicable for version 1
        WRITE_UVLC( pcVPS->getHrdOpSetIdx( i ),                "hrd_layer_set_idx" );
        if( i > 0 )
        {
          WRITE_FLAG( pcVPS->getCprmsPresentFlag( i ) ? 1 : 0, "cprms_present_flag[i]" );
        }
        codeHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
      }
    }
  }
  WRITE_FLAG( 0,                     "vps_extension_flag" );

  //future extensions here..
  xWriteRbspTrailingBits();
}

Void TEncCavlc::codeSliceHeader         ( TComSlice* pcSlice )
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceSliceHeaderEnc();
#else
  xTraceSliceHeader();
#endif
#endif

  const ChromaFormat format                = pcSlice->getSPS()->getChromaFormatIdc();
  const UInt         numberValidComponents = getNumberValidComponents(format);
  const Bool         chromaEnabled         = isChromaEnabled(format);

  //calculate number of bits required for slice address
  Int maxSliceSegmentAddress = pcSlice->getPic()->getNumberOfCtusInFrame();
  Int bitsSliceSegmentAddress = 0;
  while(maxSliceSegmentAddress>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }
  const Int ctuTsAddress = pcSlice->getSliceSegmentCurStartCtuTsAddr();

  //write slice address
  const Int sliceSegmentRsAddress = pcSlice->getPic()->getPicSym()->getCtuTsToRsAddrMap(ctuTsAddress);

  WRITE_FLAG( sliceSegmentRsAddress==0, "first_slice_segment_in_pic_flag" );
  if ( pcSlice->getRapPicFlag() )
  {
    WRITE_FLAG( pcSlice->getNoOutputPriorPicsFlag() ? 1 : 0, "no_output_of_prior_pics_flag" );
  }
  WRITE_UVLC( pcSlice->getPPS()->getPPSId(), "slice_pic_parameter_set_id" );
  if ( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() && (sliceSegmentRsAddress!=0) )
  {
    WRITE_FLAG( pcSlice->getDependentSliceSegmentFlag() ? 1 : 0, "dependent_slice_segment_flag" );
  }
  if(sliceSegmentRsAddress>0)
  {
    WRITE_CODE( sliceSegmentRsAddress, bitsSliceSegmentAddress, "slice_segment_address" );
  }
  if ( !pcSlice->getDependentSliceSegmentFlag() )
  {
    for (Int i = 0; i < pcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++)
    {
      WRITE_FLAG(0, "slice_reserved_flag[]");
    }

    WRITE_UVLC( pcSlice->getSliceType(),       "slice_type" );

    if( pcSlice->getPPS()->getOutputFlagPresentFlag() )
    {
      WRITE_FLAG( pcSlice->getPicOutputFlag() ? 1 : 0, "pic_output_flag" );
    }

    if( !pcSlice->getIdrPicFlag() )
    {
      Int picOrderCntLSB = (pcSlice->getPOC()-pcSlice->getLastIDR()+(1<<pcSlice->getSPS()->getBitsForPOC())) & ((1<<pcSlice->getSPS()->getBitsForPOC())-1);
      WRITE_CODE( picOrderCntLSB, pcSlice->getSPS()->getBitsForPOC(), "slice_pic_order_cnt_lsb");
      const TComReferencePictureSet* rps = pcSlice->getRPS();

      // check for bitstream restriction stating that:
      // If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
      // Ideally this process should not be repeated for each slice in a picture
      if (pcSlice->isIRAP())
      {
        for (Int picIdx = 0; picIdx < rps->getNumberOfPictures(); picIdx++)
        {
          assert (!rps->getUsed(picIdx));
        }
      }

      if(pcSlice->getRPSidx() < 0)
      {
        WRITE_FLAG( 0, "short_term_ref_pic_set_sps_flag");
        codeShortTermRefPicSet( rps, true, pcSlice->getSPS()->getRPSList()->getNumberOfReferencePictureSets());
      }
      else
      {
        WRITE_FLAG( 1, "short_term_ref_pic_set_sps_flag");
        Int numBits = 0;
        while ((1 << numBits) < pcSlice->getSPS()->getRPSList()->getNumberOfReferencePictureSets())
        {
          numBits++;
        }
        if (numBits > 0)
        {
          WRITE_CODE( pcSlice->getRPSidx(), numBits, "short_term_ref_pic_set_idx" );
        }
      }
      if(pcSlice->getSPS()->getLongTermRefsPresent())
      {
        Int numLtrpInSH = rps->getNumberOfLongtermPictures();
        Int ltrpInSPS[MAX_NUM_REF_PICS];
        Int numLtrpInSPS = 0;
        UInt ltrpIndex;
        Int counter = 0;
        // WARNING: The following code only works only if a matching long-term RPS is 
        //          found in the SPS for ALL long-term pictures
        //          The problem is that the SPS coded long-term pictures are moved to the
        //          beginning of the list which causes a mismatch when no reference picture
        //          list reordering is used
        //          NB: Long-term coding is currently not supported in general by the HM encoder
        for(Int k = rps->getNumberOfPictures()-1; k > rps->getNumberOfPictures()-rps->getNumberOfLongtermPictures()-1; k--)
        {
          if (findMatchingLTRP(pcSlice, &ltrpIndex, rps->getPOC(k), rps->getUsed(k)))
          {
            ltrpInSPS[numLtrpInSPS] = ltrpIndex;
            numLtrpInSPS++;
          }
          else
          {
            counter++;
          }
        }
        numLtrpInSH -= numLtrpInSPS;
        // check that either all long-term pictures are coded in SPS or in slice header (no mixing)
        assert (numLtrpInSH==0 || numLtrpInSPS==0); 

        Int bitsForLtrpInSPS = 0;
        while (pcSlice->getSPS()->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        if (pcSlice->getSPS()->getNumLongTermRefPicSPS() > 0)
        {
          WRITE_UVLC( numLtrpInSPS, "num_long_term_sps");
        }
        WRITE_UVLC( numLtrpInSH, "num_long_term_pics");
        // Note that the LSBs of the LT ref. pic. POCs must be sorted before.
        // Not sorted here because LT ref indices will be used in setRefPicList()
        Int prevDeltaMSB = 0, prevLSB = 0;
        Int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
        counter = 0;
        // Warning: If some pictures are moved to ltrpInSPS, i is referring to a wrong index 
        //          (mapping would be required)
        for(Int i=rps->getNumberOfPictures()-1 ; i > offset-1; i--, counter++)
        {
          if (counter < numLtrpInSPS)
          {
            if (bitsForLtrpInSPS > 0)
            {
              WRITE_CODE( ltrpInSPS[counter], bitsForLtrpInSPS, "lt_idx_sps[i]");
            }
          }
          else
          {
