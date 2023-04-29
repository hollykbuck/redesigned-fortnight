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

/** \file     TDecCAVLC.cpp
\brief    CAVLC decoder class
*/

#include "TDecCAVLC.h"
#include "SEIread.h"
#include "TDecSlice.h"
#include "TLibCommon/TComChromaFormat.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif
#include "TDecConformance.h"

//! \ingroup TLibDecoder
//! \{

#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
Void  xTraceVPSHeaderDec ()
#else
Void  xTraceVPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTraceSPSHeaderDec()
#else
Void  xTraceSPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" );
}

#if MCTS_EXTRACTION
Void  xTracePPSHeaderDec()
#else
Void  xTracePPSHeader()
#endif
{
  fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n");
}

#if MCTS_EXTRACTION
Void  xTraceSliceHeaderDec()
#else
Void  xTraceSliceHeader()
#endif
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc::TDecCavlc()
{
}

TDecCavlc::~TDecCavlc()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecCavlc::parseShortTermRefPicSet( TComSPS* sps, TComReferencePictureSet* rps, Int idx )
{
  UInt code;
  UInt interRPSPred;
  if (idx > 0)
  {
    READ_FLAG(interRPSPred, "inter_ref_pic_set_prediction_flag");  rps->setInterRPSPrediction(interRPSPred);
  }
  else
  {
    interRPSPred = false;
    rps->setInterRPSPrediction(false);
  }

  if (interRPSPred)
  {
    UInt bit;
    if(idx == sps->getRPSList()->getNumberOfReferencePictureSets())
    {
      READ_UVLC(code, "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }
    else
    {
      code = 0;
    }
#if MCTS_EXTRACTION 
    rps->setDeltaRIdxMinus1(code); // when -1, reference picture set explicitly signalled in slice header, otherwise identify the PPS RPS; added for MCTS extraction
#endif
    assert(code <= idx-1); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
    Int rIdx =  idx - 1 - code;
    assert (rIdx <= idx-1 && rIdx >= 0); // Made assert tighter; if rIdx = idx then prediction is done from itself. rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    TComReferencePictureSet*   rpsRef = sps->getRPSList()->getReferencePictureSet(rIdx);
    Int k = 0, k0 = 0, k1 = 0;
    READ_CODE(1, bit, "delta_rps_sign"); // delta_RPS_sign
    READ_UVLC(code, "abs_delta_rps_minus1");  // absolute delta RPS minus 1
    Int deltaRPS = (1 - 2 * bit) * (code + 1); // delta_RPS
#if MCTS_EXTRACTION
    rps->setDeltaRPS(deltaRPS); // when -1, reference picture set explicitly signalled in slice header, otherwise identify the PPS RPS; added for MCTS extraction
#endif
    for(Int j=0 ; j <= rpsRef->getNumberOfPictures(); j++)
    {
      READ_CODE(1, bit, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      Int refIdc = bit;
      if (refIdc == 0)
      {
        READ_CODE(1, bit, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit<<1; //second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if (refIdc == 1 || refIdc == 2)
      {
        Int deltaPOC = deltaRPS + ((j < rpsRef->getNumberOfPictures())? rpsRef->getDeltaPOC(j) : 0);
        rps->setDeltaPOC(k, deltaPOC);
        rps->setUsed(k, (refIdc == 1));

        if (deltaPOC < 0)
        {
          k0++;
        }
        else
        {
          k1++;
        }
        k++;
      }
      rps->setRefIdc(j,refIdc);
    }
    rps->setNumRefIdc(rpsRef->getNumberOfPictures()+1);
    rps->setNumberOfPictures(k);
    rps->setNumberOfNegativePictures(k0);
    rps->setNumberOfPositivePictures(k1);
    rps->sortDeltaPOC();
  }
  else
  {
    READ_UVLC(code, "num_negative_pics");           rps->setNumberOfNegativePictures(code);
    READ_UVLC(code, "num_positive_pics");           rps->setNumberOfPositivePictures(code);
    Int prev = 0;
    Int poc;
    for(Int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s0_minus1");
      poc = prev-code-1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s0_flag");  rps->setUsed(j,code);
    }
    prev = 0;
    for(Int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s1_minus1");
      poc = prev+code+1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s1_flag");  rps->setUsed(j,code);
    }
    rps->setNumberOfPictures(rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures());
  }
#if PRINT_RPS_INFO
  rps->printDeltaPOC();
#endif
}

Void TDecCavlc::parsePPS(TComPPS* pcPPS)
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTracePPSHeaderDec ();
#else
  xTracePPSHeader();
#endif
#endif
  UInt  uiCode;

  Int   iCode;

  READ_UVLC_CHK( uiCode, "pps_pic_parameter_set_id", 0, 63);
  assert(uiCode <= 63);
  pcPPS->setPPSId (uiCode);

  READ_UVLC_CHK( uiCode, "pps_seq_parameter_set_id", 0, 15);
  assert(uiCode <= 15);
  pcPPS->setSPSId (uiCode);

  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag"    );    pcPPS->setDependentSliceSegmentsEnabledFlag   ( uiCode == 1 );

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);

  READ_FLAG( uiCode, "sign_data_hiding_enabled_flag" );               pcPPS->setSignDataHidingEnabledFlag( uiCode );

  READ_FLAG( uiCode,   "cabac_init_present_flag" );                   pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC_CHK(uiCode, "num_ref_idx_l0_default_active_minus1", 0, 14);
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC_CHK(uiCode, "num_ref_idx_l1_default_active_minus1", 0, 14);
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

  READ_SVLC_CHK(iCode, "init_qp_minus26", std::numeric_limits<Int>::min(), 25 );  pcPPS->setPicInitQPMinus26(iCode);
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );              pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip ( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  if( pcPPS->getUseDQP() )
  {
    READ_UVLC( uiCode, "diff_cu_qp_delta_depth" );
    pcPPS->setMaxCuDQPDepth( uiCode );
  }
  else
  {
    pcPPS->setMaxCuDQPDepth( 0 );
  }
  READ_SVLC_CHK( iCode, "pps_cb_qp_offset", -12, 12);
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cb) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cb) <=  12 );

  READ_SVLC_CHK( iCode, "pps_cr_qp_offset", -12, 12);
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cr) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cr) <=  12 );

  assert(MAX_NUM_COMPONENT<=3);

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enabled_flag");
  pcPPS->setTransquantBypassEnabledFlag(uiCode ? true : false);
  READ_FLAG( uiCode, "tiles_enabled_flag"               );    pcPPS->setTilesEnabledFlag            ( uiCode == 1 );
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );    pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if( pcPPS->getTilesEnabledFlag() )
  {
    READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );  
    READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );  
    READ_FLAG ( uiCode, "uniform_spacing_flag" );                   pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    const UInt tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const UInt tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();
 
    if ( !pcPPS->getTileUniformSpacingFlag())
    {
      if (tileColumnsMinus1 > 0)
      {
        std::vector<Int> columnWidth(tileColumnsMinus1);
        for(UInt i = 0; i < tileColumnsMinus1; i++)
        { 
          READ_UVLC( uiCode, "column_width_minus1" );  
          columnWidth[i] = uiCode+1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<Int> rowHeight (tileRowsMinus1);
        for(UInt i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC( uiCode, "row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
    }
    assert ((tileColumnsMinus1 + tileRowsMinus1) != 0);
    READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );     pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
  }
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );        pcPPS->setPPSDeblockingFilterDisabledFlag(uiCode ? true : false );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if(pcPPS->getScalingListPresentFlag ())
  {
    parseScalingList( &(pcPPS->getScalingList()) );
  }

  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);

  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
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

    Bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
          case PPS_EXT__REXT:
            {
              TComPPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
              assert(!bSkipTrailingExtensionBits);

              if (pcPPS->getUseTransformSkip())
              {
                READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2");
                ppsRangeExtension.setLog2MaxTransformSkipBlockSize(uiCode+2);
              }

              READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
              ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

              READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag");
              if (uiCode == 0)
              {
                ppsRangeExtension.clearChromaQpOffsetList();
                ppsRangeExtension.setDiffCuChromaQpOffsetDepth(0);
              }
              else
              {
                READ_UVLC(uiCode, "diff_cu_chroma_qp_offset_depth"); ppsRangeExtension.setDiffCuChromaQpOffsetDepth(uiCode);
                UInt tableSizeMinus1 = 0;
                READ_UVLC_CHK(tableSizeMinus1, "chroma_qp_offset_list_len_minus1", 0, MAX_QP_OFFSET_LIST_SIZE-1);
                assert(tableSizeMinus1 < MAX_QP_OFFSET_LIST_SIZE);

                for (Int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
                {
                  Int cbOffset;
                  Int crOffset;
                  READ_SVLC_CHK(cbOffset, "cb_qp_offset_list[i]", -12, 12);
                  assert(cbOffset >= -12 && cbOffset <= 12);
                  READ_SVLC_CHK(crOffset, "cr_qp_offset_list[i]", -12, 12);
                  assert(crOffset >= -12 && crOffset <= 12);
                  // table uses +1 for index (see comment inside the function)
                  ppsRangeExtension.setChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1, cbOffset, crOffset);
                }
                assert(ppsRangeExtension.getChromaQpOffsetListLen() == tableSizeMinus1 + 1);
              }

              READ_UVLC( uiCode, "log2_sao_offset_scale_luma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA, uiCode);
              READ_UVLC( uiCode, "log2_sao_offset_scale_chroma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, uiCode);
            }
            break;
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

Void  TDecCavlc::parseVUI(TComVUI* pcVUI, TComSPS *pcSPS)
{
#if ENC_DEC_TRACE
  fprintf( g_hTrace, "----------- vui_parameters -----------\n");
#endif
  UInt  uiCode;

  READ_FLAG(     uiCode, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(uiCode);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_CODE(8, uiCode, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(uiCode);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, uiCode, "sar_width");                             pcVUI->setSarWidth(uiCode);
      READ_CODE(16, uiCode, "sar_height");                            pcVUI->setSarHeight(uiCode);
    }
  }

  READ_FLAG(     uiCode, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(uiCode);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   uiCode, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(uiCode);
  }

  READ_FLAG(     uiCode, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(uiCode);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_CODE(3, uiCode, "video_format");                             pcVUI->setVideoFormat(uiCode);
    READ_FLAG(   uiCode, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(uiCode);
    READ_FLAG(   uiCode, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(uiCode);
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      READ_CODE(8, uiCode, "colour_primaries");                       pcVUI->setColourPrimaries(uiCode);
      READ_CODE(8, uiCode, "transfer_characteristics");               pcVUI->setTransferCharacteristics(uiCode);
      READ_CODE(8, uiCode, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(uiCode);
    }
  }

  READ_FLAG(     uiCode, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(uiCode);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    READ_UVLC(   uiCode, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(uiCode);
    READ_UVLC(   uiCode, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(uiCode);
  }

  READ_FLAG(     uiCode, "neutral_chroma_indication_flag");           pcVUI->setNeutralChromaIndicationFlag(uiCode);

  READ_FLAG(     uiCode, "field_seq_flag");                           pcVUI->setFieldSeqFlag(uiCode);

  READ_FLAG(uiCode, "frame_field_info_present_flag");                 pcVUI->setFrameFieldInfoPresentFlag(uiCode);

  READ_FLAG(     uiCode, "default_display_window_flag");
  if (uiCode != 0)
  {
    Window &defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC(   uiCode, "def_disp_win_left_offset" );                defDisp.setWindowLeftOffset  ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_right_offset" );               defDisp.setWindowRightOffset ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_top_offset" );                 defDisp.setWindowTopOffset   ( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_bottom_offset" );              defDisp.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
  }

  TimingInfo *timingInfo = pcVUI->getTimingInfo();
  READ_FLAG(       uiCode, "vui_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vui_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vui_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vui_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_FLAG(     uiCode, "vui_hrd_parameters_present_flag");        pcVUI->setHrdParametersPresentFlag(uiCode);
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG(     uiCode, "bitstream_restriction_flag");               pcVUI->setBitstreamRestrictionFlag(uiCode);
  if (pcVUI->getBitstreamRestrictionFlag())
  {
    READ_FLAG(   uiCode, "tiles_fixed_structure_flag");               pcVUI->setTilesFixedStructureFlag(uiCode);
    READ_FLAG(   uiCode, "motion_vectors_over_pic_boundaries_flag");  pcVUI->setMotionVectorsOverPicBoundariesFlag(uiCode);
    READ_FLAG(   uiCode, "restricted_ref_pic_lists_flag");            pcVUI->setRestrictedRefPicListsFlag(uiCode);
    READ_UVLC(   uiCode, "min_spatial_segmentation_idc");             pcVUI->setMinSpatialSegmentationIdc(uiCode);
    assert(uiCode < 4096);
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_min_cu_denom" );               pcVUI->setMaxBitsPerMinCuDenom(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_horizontal" );           pcVUI->setLog2MaxMvLengthHorizontal(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_vertical" );             pcVUI->setLog2MaxMvLengthVertical(uiCode);
  }
}

Void TDecCavlc::parseHrdParameters(TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1)
{
  UInt  uiCode;
  if( commonInfPresentFlag )
  {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" ); hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1"  ); hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );                       hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );                       hrd->setCpbSizeScale( uiCode );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );                  hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" ); hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );       hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
  }
  Int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, 0 ); // Infered to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 ); // Infered to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, uiCode );
    }
    else
    {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, uiCode );
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );       hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );       hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false  );
        }
      }
    }
  }
}

Void TDecCavlc::parseSPS(TComSPS* pcSPS)
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceSPSHeaderDec ();
#else
  xTraceSPSHeader ();
#endif
#endif

  UInt  uiCode;
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");             pcSPS->setVPSId        ( uiCode );
  READ_CODE_CHK( 3,  uiCode, "sps_max_sub_layers_minus1", 0, 6 );   pcSPS->setMaxTLayers   ( uiCode+1 );
  assert(uiCode <= 6);

  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    TDecConformanceCheck::checkRange(uiCode, "sps_temporal_id_nesting_flag", 1U, 1U);
    assert( uiCode == 1 );
  }

  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  READ_UVLC_CHK( uiCode, "sps_seq_parameter_set_id", 0, 15 );           pcSPS->setSPSId( uiCode );
  assert(uiCode <= 15);

  READ_UVLC_CHK(     uiCode, "chroma_format_idc", 0, 3 );               pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
  assert(uiCode <= 3);

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG_CHK(     uiCode, "separate_colour_plane_flag", 0, 0);
    assert(uiCode == 0);
  }

  // pic_width_in_luma_samples and pic_height_in_luma_samples needs conformance checking - multiples of MinCbSizeY
  READ_UVLC_CHK (    uiCode, "pic_width_in_luma_samples", 1, std::numeric_limits<UInt>::max()  );  pcSPS->setPicWidthInLumaSamples ( uiCode    );
  READ_UVLC_CHK (    uiCode, "pic_height_in_luma_samples", 1, std::numeric_limits<UInt>::max() );  pcSPS->setPicHeightInLumaSamples( uiCode    );
  READ_FLAG(     uiCode, "conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
    const UInt subWidthC  = TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() );
    const UInt subHeightC = TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() );
    READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * subWidthC );
    READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * subWidthC );
    READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * subHeightC );
    READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * subHeightC );
    TDecConformanceCheck::checkRange<UInt>(conf.getWindowLeftOffset()+conf.getWindowRightOffset(), "conformance window width in pixels", 0, pcSPS->getPicWidthInLumaSamples()-1);
  }

  READ_UVLC_CHK(     uiCode, "bit_depth_luma_minus8", 0, 8 );
  assert(uiCode <= 8);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);
  const UInt forceDecodeBitDepth = pcSPS->getForceDecodeBitDepth();
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
  assert(uiCode <= 8);
#endif
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*uiCode) );
#endif

  READ_UVLC_CHK( uiCode,    "bit_depth_chroma_minus8", 0, 8 );
  assert(uiCode <= 8);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
  assert(uiCode <= 8);
#endif
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_CHROMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*uiCode) );
#endif

  READ_UVLC_CHK( uiCode,    "log2_max_pic_order_cnt_lsb_minus4", 0, 12 );   pcSPS->setBitsForPOC( 4 + uiCode );

  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

  for(UInt i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
    READ_UVLC ( uiCode, "sps_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics(uiCode, i);
    READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncreasePlus1( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
      {
        pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
        pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
        pcSPS->setMaxLatencyIncreasePlus1(pcSPS->getMaxLatencyIncreasePlus1(0), i);
      }
      break;
    }
  }

  const UInt maxLog2CtbSize = TDecConformanceCheck::getMaxLog2CtbSize(*(pcSPS->getPTL()));
  const UInt minLog2CtbSize = TDecConformanceCheck::getMinLog2CtbSize(*(pcSPS->getPTL()));
  READ_UVLC_CHK( uiCode, "log2_min_luma_coding_block_size_minus3", 0, maxLog2CtbSize-3 );
  assert(uiCode <= maxLog2CtbSize-3);
  Int minCbLog2SizeY = uiCode + 3;
  pcSPS->setLog2MinCodingBlockSize(minCbLog2SizeY);

  // Difference + log2MinCUSize must be <= maxLog2CtbSize
  // Difference + log2MinCUSize must be >= minLog2CtbSize
  const UInt minLog2DiffMaxMinLumaCodingBlockSize = minLog2CtbSize < minCbLog2SizeY ? 0 : minLog2CtbSize - minCbLog2SizeY;
  const UInt maxLog2DiffMaxMinLumaCodingBlockSize = maxLog2CtbSize - minCbLog2SizeY;

  READ_UVLC_CHK( uiCode, "log2_diff_max_min_luma_coding_block_size", minLog2DiffMaxMinLumaCodingBlockSize, maxLog2DiffMaxMinLumaCodingBlockSize);
  assert(uiCode >= minLog2DiffMaxMinLumaCodingBlockSize && uiCode <= maxLog2DiffMaxMinLumaCodingBlockSize);
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  
  const Int maxCUDepthDelta = uiCode;
  const Int ctbLog2SizeY = minCbLog2SizeY + maxCUDepthDelta;
  pcSPS->setMaxCUWidth  ( 1<<ctbLog2SizeY );
  pcSPS->setMaxCUHeight ( 1<<ctbLog2SizeY );
  READ_UVLC_CHK( uiCode, "log2_min_luma_transform_block_size_minus2", 0, minCbLog2SizeY-1-2 );
  const UInt minTbLog2SizeY = uiCode + 2;
  pcSPS->setQuadtreeTULog2MinSize( minTbLog2SizeY );

  //  log2_diff <= Min(CtbLog2SizeY, 5) - minTbLog2SizeY
  READ_UVLC_CHK( uiCode, "log2_diff_max_min_luma_transform_block_size", 0, min<UInt>(5U, ctbLog2SizeY) - minTbLog2SizeY );
  pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTrSize( 1<<(uiCode + pcSPS->getQuadtreeTULog2MinSize()) );

  READ_UVLC_CHK( uiCode, "max_transform_hierarchy_depth_inter", 0, ctbLog2SizeY - minTbLog2SizeY);    pcSPS->setQuadtreeTUMaxDepthInter( uiCode+1 );
  READ_UVLC_CHK( uiCode, "max_transform_hierarchy_depth_intra", 0, ctbLog2SizeY - minTbLog2SizeY);    pcSPS->setQuadtreeTUMaxDepthIntra( uiCode+1 );

  Int addCuDepth = max (0, minCbLog2SizeY - (Int)pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTotalCUDepth( maxCUDepthDelta + addCuDepth  + getMaxCUDepthOffset(pcSPS->getChromaFormatIdc(), pcSPS->getQuadtreeTULog2MinSize()) );

  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
    if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
  }
  READ_FLAG( uiCode, "amp_enabled_flag" );                          pcSPS->setUseAMP( uiCode );
  READ_FLAG( uiCode, "sample_adaptive_offset_enabled_flag" );       pcSPS->setUseSAO ( uiCode ? true : false );

  READ_FLAG( uiCode, "pcm_enabled_flag" ); pcSPS->setUsePCM( uiCode ? true : false );
  if( pcSPS->getUsePCM() )
  {
#if O0043_BEST_EFFORT_DECODING
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_luma_minus1",   0, pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA) );    pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1", 0, pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA) );    pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
#else
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_luma_minus1",   0, pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1", 0, pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) );        pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
#endif
    READ_UVLC_CHK( uiCode, "log2_min_pcm_luma_coding_block_size_minus3", std::min<UInt>(minCbLog2SizeY, 5 )-3, std::min<UInt>(ctbLog2SizeY, 5)-3);
    const UInt log2MinIpcmCbSizeY = uiCode+3;
    pcSPS->setPCMLog2MinSize (log2MinIpcmCbSizeY);
    READ_UVLC_CHK( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size", 0, (std::min<UInt>(ctbLog2SizeY,5) - log2MinIpcmCbSizeY) );
    pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_UVLC_CHK( uiCode, "num_short_term_ref_pic_sets", 0, 64 );
  assert(uiCode <= 64);
  pcSPS->createRPSList(uiCode);

  TComRPSList* rpsList = pcSPS->getRPSList();
  TComReferencePictureSet* rps;

  for(UInt i=0; i< rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC_CHK( uiCode, "num_long_term_ref_pics_sps", 0, 32 );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (UInt k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }
  READ_FLAG( uiCode, "sps_temporal_mvp_enabled_flag" );           pcSPS->setSPSTemporalMVPEnabledFlag(uiCode);

  READ_FLAG( uiCode, "strong_intra_smoothing_enable_flag" );      pcSPS->setUseStrongIntraSmoothing(uiCode);

  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  READ_FLAG( uiCode, "sps_extension_present_flag");
  if (uiCode)
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
    Bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
          case SPS_EXT__REXT:
            assert(!bSkipTrailingExtensionBits);
            {
              TComSPSRExt &spsRangeExtension = pcSPS->getSpsRangeExtension();
              READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");     spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0);
              READ_FLAG( uiCode, "transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "extended_precision_processing_flag");       spsRangeExtension.setExtendedPrecisionProcessingFlag (uiCode != 0);
              READ_FLAG( uiCode, "intra_smoothing_disabled_flag");            spsRangeExtension.setIntraSmoothingDisabledFlag      (uiCode != 0);
              READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");      spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");      spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
            }
            break;
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }

  xReadRbspTrailingBits();
}

Void TDecCavlc::parseVPS(TComVPS* pcVPS)
{
#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceVPSHeaderDec ();
#else
  xTraceVPSHeader();
#endif
#endif
  UInt  uiCode;

  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );         pcVPS->setVPSId( uiCode );
  READ_FLAG( uiCode,      "vps_base_layer_internal_flag" );       assert(uiCode == 1);
  READ_FLAG( uiCode,      "vps_base_layer_available_flag" );      assert(uiCode == 1);
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 );    assert(uiCode+1 <= MAX_TLAYER);
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  assert (pcVPS->getMaxTLayers()>1||pcVPS->getTemporalNestingFlag());
  READ_CODE( 16, uiCode,  "vps_reserved_0xffff_16bits" );         assert(uiCode == 0xffff);
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(UInt i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_max_num_reorder_pics[i]" );            pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode,  "vps_max_latency_increase_plus1[i]" );      pcVPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcVPS->getMaxTLayers()-1; i++)
      {
        pcVPS->setMaxDecPicBuffering(pcVPS->getMaxDecPicBuffering(0), i);
        pcVPS->setNumReorderPics(pcVPS->getNumReorderPics(0), i);
        pcVPS->setMaxLatencyIncrease(pcVPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

  assert( pcVPS->getNumHrdParameters() < MAX_VPS_OP_SETS_PLUS1 );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );                        pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

  TimingInfo *timingInfo = pcVPS->getTimingInfo();
  READ_FLAG(       uiCode, "vps_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vps_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vps_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vps_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_UVLC( uiCode, "vps_num_hrd_parameters" );                  pcVPS->setNumHrdParameters( uiCode );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      pcVPS->createHrdParamBuffer();
    }
    for( UInt i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
    {
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );                  pcVPS->setHrdOpSetIdx( uiCode, i );
      if( i > 0 )
      {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );               pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      }
      else
      {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
    }
  }

  READ_FLAG( uiCode,  "vps_extension_flag" );
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}

Void TDecCavlc::parseSliceHeader (TComSlice* pcSlice, ParameterSetManager *parameterSetManager, const Int prevTid0POC)
{
  UInt  uiCode;
  Int   iCode;

#if ENC_DEC_TRACE
#if MCTS_EXTRACTION
  xTraceSliceHeaderDec();
#else
  xTraceSliceHeader();
#endif
#endif
  TComPPS* pps = NULL;
  TComSPS* sps = NULL;

  UInt firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
  if( pcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  pcSlice->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(uiCode);
  //!KS: need to add error handling code here, if PPS is not available
  assert(pps!=0);
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  assert(sps!=0);

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const UInt numValidComp=getNumberValidComponents(chFmt);
  const Bool bChroma=(chFmt!=CHROMA_400);

  if( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ))
  {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );       pcSlice->setDependentSliceSegmentFlag(uiCode ? true : false);
  }
  else
  {
    pcSlice->setDependentSliceSegmentFlag(false);
  }
  Int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
  UInt sliceSegmentAddress = 0;
  Int bitsSliceSegmentAddress = 0;
  while(numCTUs>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }

  if(!firstSliceSegmentInPic)
  {
    READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" );
  }
  //set uiCode to equal slice start address (or dependent slice start address)
  pcSlice->setSliceSegmentCurStartCtuTsAddr( sliceSegmentAddress );// this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
  pcSlice->setSliceSegmentCurEndCtuTsAddr(numCTUs);                // Set end as the last CTU of the picture.

  if (!pcSlice->getDependentSliceSegmentFlag())
  {
    pcSlice->setSliceCurStartCtuTsAddr(sliceSegmentAddress); // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr(numCTUs);
  }

  if(!pcSlice->getDependentSliceSegmentFlag())
  {
    for (Int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_flag[]"); // ignored
    }

    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    pcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

    if( pcSlice->getIdrPicFlag() )
    {
      pcSlice->setPOC(0);
      TComReferencePictureSet* rps = pcSlice->getLocalRPS();
      (*rps)=TComReferencePictureSet();
      pcSlice->setRPS(rps);
    }
    else
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      Int iPOClsb = uiCode;
      Int iPrevPOC = prevTid0POC;
      Int iMaxPOClsb = 1<< sps->getBitsForPOC();
      Int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
      Int iPrevPOCmsb = iPrevPOC-iPrevPOClsb;
      Int iPOCmsb;
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb )  >=  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( (iPOClsb  >  iPrevPOClsb )  && ( (iPOClsb - iPrevPOClsb )  >  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      pcSlice->setPOC              (iPOCmsb+iPOClsb);

      TComReferencePictureSet* rps;
      rps = pcSlice->getLocalRPS();
      (*rps)=TComReferencePictureSet();

      pcSlice->setRPS(rps);
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
#if MCTS_EXTRACTION
        pcSlice->setRPSidx(-1); // when -1, reference picture set explicitly signalled in slice header, otherwise identify the PPS RPS; added for MCTS extraction
#endif
      }
      else // use reference to short-term reference picture set in PPS
      {
        Int numBits = 0;
        while ((1 << numBits) < sps->getRPSList()->getNumberOfReferencePictureSets())
        {
          numBits++;
        }
        if (numBits > 0)
        {
          READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx");
#if MCTS_EXTRACTION
          pcSlice->setRPSidx(uiCode); // when -1, reference picture set explicitly signalled in slice header, otherwise identify the PPS RPS; added for MCTS extraction
#endif
        }
        else
        {
          uiCode = 0;
       
        }
        *rps = *(sps->getRPSList()->getReferencePictureSet(uiCode));
      }
      if(sps->getLongTermRefsPresent())
      {
        Int offset = rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures();
        UInt numOfLtrp = 0;
        UInt numLtrpInSPS = 0;
        if (sps->getNumLongTermRefPicSPS() > 0)
        {
          READ_UVLC( uiCode, "num_long_term_sps");
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures(numOfLtrp);
        }
        Int bitsForLtrpInSPS = 0;
        while (sps->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures(numOfLtrp);
        Int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        Int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for(Int j=offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < numOfLtrp; j--, k++)
        {
          Int pocLsbLt;
          if (k < numLtrpInSPS)
          {
            uiCode = 0;
            if (bitsForLtrpInSPS > 0)
            {
              READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
            }
            Bool usedByCurrFromSPS=sps->getUsedByCurrPicLtSPSFlag(uiCode);

            pocLsbLt = sps->getLtRefPicPocLsbSps(uiCode);
            rps->setUsed(j,usedByCurrFromSPS);
          }
          else
          {
            READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
          }
          READ_FLAG(uiCode,"delta_poc_msb_present_flag");
          Bool mSBPresentFlag = uiCode ? true : false;
          if(mSBPresentFlag)
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            Bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(numOfLtrp-numLtrpInSPS)-1) )
            {
              deltaFlag = true;
            }
            if(deltaFlag)
            {
              deltaPocMSBCycleLT = uiCode;
            }
            else
            {
              deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
            }

            Int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
                                        - iPOClsb + pocLsbLt;
            rps->setPOC     (j, pocLTCurr);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLTCurr);
            rps->setCheckLTMSBPresent(j,true);
          }
          else
          {
            rps->setPOC     (j, pocLsbLt);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLsbLt);
            rps->setCheckLTMSBPresent(j,false);

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if( j == offset+(numOfLtrp-numLtrpInSPS)-1 )
            {
              deltaPocMSBCycleLT = 0;
            }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures(offset);
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps = pcSlice->getLocalRPS();
        (*rps)=TComReferencePictureSet();
        pcSlice->setRPS(rps);
      }
      if (sps->getSPSTemporalMVPEnabledFlag())
      {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      }
      else
      {
        pcSlice->setEnableTMVPFlag(false);
      }
    }
    if(sps->getUseSAO())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (Bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (Bool)uiCode);
      }
    }

    if (pcSlice->getIdrPicFlag())
    {
      pcSlice->setEnableTMVPFlag(false);
    }
    if (!pcSlice->isIntra())
    {

      READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
      if (uiCode)
      {
        READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if (pcSlice->isInterB())
        {
          READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
        }
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
        if (pcSlice->isInterB())
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
        }
      }
    }
    // }
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if(!pcSlice->isIntra())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL0( 0 );
      }
