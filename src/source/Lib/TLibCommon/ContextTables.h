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

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_NUM_CTX_MOD             512       ///< maximum number of supported contexts

#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended

#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#define NUM_INTRA_PREDICT_CTX         1       ///< number of context models for intra prediction

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference
#define NUM_CHROMA_QP_ADJ_FLAG_CTX    1       ///< number of context models for chroma_qp_adjustment_flag
#define NUM_CHROMA_QP_ADJ_IDC_CTX     1       ///< number of context models for chroma_qp_adjustment_idc

#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE
#define NUM_EXPLICIT_RDPCM_FLAG_CTX   1       ///< number of context models for the flag which specifies whether to use RDPCM on inter coded residues
#define NUM_EXPLICIT_RDPCM_DIR_CTX    1       ///< number of context models for the flag which specifies which RDPCM direction is used on inter coded residues

//--------------------------------------------------------------------------------------------------

// context size definitions for significance map

#define NUM_SIG_FLAG_CTX_LUMA        28       ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA      16       ///< number of context models for chroma sig flag

//                                                                                                           |----Luma-----|  |---Chroma----|
static const UInt significanceMapContextSetStart         [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 21, 27}, {0,  9, 12, 15} };
static const UInt significanceMapContextSetSize          [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9, 12,  6,  1}, {9,  3,  3,  1} };
static const UInt nonDiagonalScan8x8ContextOffset        [MAX_NUM_CHANNEL_TYPE]                          = {  6,               0              };
static const UInt notFirstGroupNeighbourhoodContextOffset[MAX_NUM_CHANNEL_TYPE]                          = {  3,               0              };

//------------------

#define NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4  3
#define NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4  1

//------------------

#define FIRST_SIG_FLAG_CTX_LUMA                   0
#define FIRST_SIG_FLAG_CTX_CHROMA     (FIRST_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_LUMA)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_CHROMA)       ///< number of context models for sig flag

//--------------------------------------------------------------------------------------------------

// context size definitions for last significant coefficient position

#define NUM_CTX_LAST_FLAG_SETS         2

#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position

//--------------------------------------------------------------------------------------------------

// context size definitions for greater-than-one and greater-than-two maps

#define NUM_ONE_FLAG_CTX_PER_SET       4      ///< number of context models for greater than 1 flag in a set
#define NUM_ABS_FLAG_CTX_PER_SET       1      ///< number of context models for greater than 2 flag in a set

//------------------

#define NUM_CTX_SETS_LUMA              4      ///< number of context model sets for luminance
#define NUM_CTX_SETS_CHROMA            2      ///< number of context model sets for combined chrominance

#define FIRST_CTX_SET_LUMA             0      ///< index of first luminance context set

//------------------

#define NUM_ONE_FLAG_CTX_LUMA         (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA       (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 1 flag of chroma

#define NUM_ABS_FLAG_CTX_LUMA         (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA       (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 2 flag of chroma

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA + NUM_ONE_FLAG_CTX_CHROMA)        ///< number of context models for greater than 1 flag
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA + NUM_ABS_FLAG_CTX_CHROMA)        ///< number of context models for greater than 2 flag

#define FIRST_CTX_SET_CHROMA          (FIRST_CTX_SET_LUMA + NUM_CTX_SETS_LUMA)                 ///< index of first chrominance context set

//--------------------------------------------------------------------------------------------------

// context size definitions for CBF

#define NUM_QT_CBF_CTX_SETS           2

#define NUM_QT_CBF_CTX_PER_SET        5       ///< number of context models for QT CBF

#define FIRST_CBF_CTX_LUMA            0       ///< index of first luminance CBF context

#define FIRST_CBF_CTX_CHROMA          (FIRST_CBF_CTX_LUMA + NUM_QT_CBF_CTX_PER_SET)  ///< index of first chrominance CBF context


//--------------------------------------------------------------------------------------------------

#define NUM_MVP_IDX_CTX               1       ///< number of context models for MVP index

#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping

#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1

#define NUM_CROSS_COMPONENT_PREDICTION_CTX 10

#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'


// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
INIT_CU_TRANSQUANT_BYPASS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
{
  { 154 },
  { 154 },
  { 154 },
};

// initial probability for split flag
static const UChar
INIT_SPLIT_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SPLIT_FLAG_CTX] =
{
  { 107,  139,  126, },
  { 107,  139,  126, },
  { 139,  141,  157, },
};

static const UChar
INIT_SKIP_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SKIP_FLAG_CTX] =
{
  { 197,  185,  201, },
  { 197,  185,  201, },
  { CNU,  CNU,  CNU, },
};

static const UChar
INIT_MERGE_FLAG_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_FLAG_EXT_CTX] =
{
  { 154, },
  { 110, },
