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

/** \file     TDecSbac.cpp
    \brief    Context-adaptive entropy decoder class
*/

#include "TDecSbac.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/TComTrQuant.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
//
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a) , a
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(a)
#endif

//! \ingroup TLibDecoder
//! \{

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
#include "../TLibCommon/Debug.h"
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSbac::TDecSbac()
// new structure here
: m_pcBitstream                              ( 0 )
, m_pcTDecBinIf                              ( NULL )
, m_numContextModels                         ( 0 )
, m_cCUSplitFlagSCModel                      ( 1,             1,                      NUM_SPLIT_FLAG_CTX                   , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSkipFlagSCModel                       ( 1,             1,                      NUM_SKIP_FLAG_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeFlagExtSCModel                   ( 1,             1,                      NUM_MERGE_FLAG_EXT_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMergeIdxExtSCModel                    ( 1,             1,                      NUM_MERGE_IDX_EXT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPartSizeSCModel                       ( 1,             1,                      NUM_PART_SIZE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUPredModeSCModel                       ( 1,             1,                      NUM_PRED_MODE_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUIntraPredSCModel                      ( 1,             1,                      NUM_INTRA_PREDICT_CTX                , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUChromaPredSCModel                     ( 1,             1,                      NUM_CHROMA_PRED_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUDeltaQpSCModel                        ( 1,             1,                      NUM_DELTA_QP_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUInterDirSCModel                       ( 1,             1,                      NUM_INTER_DIR_CTX                    , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCURefPicSCModel                         ( 1,             1,                      NUM_REF_NO_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUMvdSCModel                            ( 1,             1,                      NUM_MV_RES_CTX                       , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtCbfSCModel                          ( 1,             NUM_QT_CBF_CTX_SETS,    NUM_QT_CBF_CTX_PER_SET               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUTransSubdivFlagSCModel                ( 1,             1,                      NUM_TRANS_SUBDIV_FLAG_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUQtRootCbfSCModel                      ( 1,             1,                      NUM_QT_ROOT_CBF_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigCoeffGroupSCModel                  ( 1,             2,                      NUM_SIG_CG_FLAG_CTX                  , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUSigSCModel                            ( 1,             1,                      NUM_SIG_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastX                              ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCuCtxLastY                              ( 1,             NUM_CTX_LAST_FLAG_SETS, NUM_CTX_LAST_FLAG_XY                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUOneSCModel                            ( 1,             1,                      NUM_ONE_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCUAbsSCModel                            ( 1,             1,                      NUM_ABS_FLAG_CTX                     , m_contextModels + m_numContextModels, m_numContextModels)
, m_cMVPIdxSCModel                           ( 1,             1,                      NUM_MVP_IDX_CTX                      , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoMergeSCModel                         ( 1,             1,                      NUM_SAO_MERGE_FLAG_CTX               , m_contextModels + m_numContextModels, m_numContextModels)
, m_cSaoTypeIdxSCModel                       ( 1,             1,                      NUM_SAO_TYPE_IDX_CTX                 , m_contextModels + m_numContextModels, m_numContextModels)
, m_cTransformSkipSCModel                    ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_TRANSFORMSKIP_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_CUTransquantBypassFlagSCModel            ( 1,             1,                      NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX    , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmFlagSCModel                 ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_FLAG_CTX          , m_contextModels + m_numContextModels, m_numContextModels)
, m_explicitRdpcmDirSCModel                  ( 1,             MAX_NUM_CHANNEL_TYPE,   NUM_EXPLICIT_RDPCM_DIR_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_cCrossComponentPredictionSCModel         ( 1,             1,                      NUM_CROSS_COMPONENT_PREDICTION_CTX   , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjFlagSCModel                   ( 1,             1,                      NUM_CHROMA_QP_ADJ_FLAG_CTX           , m_contextModels + m_numContextModels, m_numContextModels)
, m_ChromaQpAdjIdcSCModel                    ( 1,             1,                      NUM_CHROMA_QP_ADJ_IDC_CTX            , m_contextModels + m_numContextModels, m_numContextModels)
{
  assert( m_numContextModels <= MAX_NUM_CTX_MOD );
}

TDecSbac::~TDecSbac()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecSbac::resetEntropy(TComSlice* pSlice)
{
  SliceType sliceType  = pSlice->getSliceType();
  Int       qp         = pSlice->getSliceQp();

  if (pSlice->getPPS()->getCabacInitPresentFlag() && pSlice->getCabacInitFlag())
  {
    switch (sliceType)
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      assert(0);
      break;
    }
  }

  m_cCUSplitFlagSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_SPLIT_FLAG );
  m_cCUSkipFlagSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_SKIP_FLAG );
  m_cCUMergeFlagExtSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_MERGE_FLAG_EXT );
  m_cCUMergeIdxExtSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_MERGE_IDX_EXT );
  m_cCUPartSizeSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_PART_SIZE );
  m_cCUPredModeSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_PRED_MODE );
  m_cCUIntraPredSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_INTRA_PRED_MODE );
  m_cCUChromaPredSCModel.initBuffer               ( sliceType, qp, (UChar*)INIT_CHROMA_PRED_MODE );
  m_cCUInterDirSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_INTER_DIR );
  m_cCUMvdSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_MVD );
  m_cCURefPicSCModel.initBuffer                   ( sliceType, qp, (UChar*)INIT_REF_PIC );
  m_cCUDeltaQpSCModel.initBuffer                  ( sliceType, qp, (UChar*)INIT_DQP );
  m_cCUQtCbfSCModel.initBuffer                    ( sliceType, qp, (UChar*)INIT_QT_CBF );
  m_cCUQtRootCbfSCModel.initBuffer                ( sliceType, qp, (UChar*)INIT_QT_ROOT_CBF );
  m_cCUSigCoeffGroupSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_SIG_CG_FLAG );
  m_cCUSigSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_SIG_FLAG );
  m_cCuCtxLastX.initBuffer                        ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCuCtxLastY.initBuffer                        ( sliceType, qp, (UChar*)INIT_LAST );
  m_cCUOneSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_ONE_FLAG );
  m_cCUAbsSCModel.initBuffer                      ( sliceType, qp, (UChar*)INIT_ABS_FLAG );
  m_cMVPIdxSCModel.initBuffer                     ( sliceType, qp, (UChar*)INIT_MVP_IDX );
  m_cSaoMergeSCModel.initBuffer                   ( sliceType, qp, (UChar*)INIT_SAO_MERGE_FLAG );
  m_cSaoTypeIdxSCModel.initBuffer                 ( sliceType, qp, (UChar*)INIT_SAO_TYPE_IDX );
  m_cCUTransSubdivFlagSCModel.initBuffer          ( sliceType, qp, (UChar*)INIT_TRANS_SUBDIV_FLAG );
  m_cTransformSkipSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_TRANSFORMSKIP_FLAG );
  m_CUTransquantBypassFlagSCModel.initBuffer      ( sliceType, qp, (UChar*)INIT_CU_TRANSQUANT_BYPASS_FLAG );
  m_explicitRdpcmFlagSCModel.initBuffer           ( sliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_FLAG);
  m_explicitRdpcmDirSCModel.initBuffer            ( sliceType, qp, (UChar*)INIT_EXPLICIT_RDPCM_DIR);
  m_cCrossComponentPredictionSCModel.initBuffer   ( sliceType, qp, (UChar*)INIT_CROSS_COMPONENT_PREDICTION );
  m_ChromaQpAdjFlagSCModel.initBuffer             ( sliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_FLAG );
  m_ChromaQpAdjIdcSCModel.initBuffer              ( sliceType, qp, (UChar*)INIT_CHROMA_QP_ADJ_IDC );

  for (UInt statisticIndex = 0; statisticIndex < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS ; statisticIndex++)
  {
    m_golombRiceAdaptationStatistics[statisticIndex] = 0;
  }

  m_pcTDecBinIf->start();
}

Void TDecSbac::parseTerminatingBit( UInt& ruiBit )
{
  m_pcTDecBinIf->decodeBinTrm( ruiBit );
  if ( ruiBit == 1 )
  {
    m_pcTDecBinIf->finish();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::IncrementStatisticEP(STATS__TRAILING_BITS, m_pcBitstream->readOutTrailingBits(),0);
#else
    m_pcBitstream->readOutTrailingBits();
#endif
  }
}

Void TDecSbac::parseRemainingBytes( Bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    const UInt numberOfRemainingSubstreamBytes=m_pcBitstream->getNumBitsLeft();
    assert (numberOfRemainingSubstreamBytes == 0);
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      UInt trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        printf("Trailing byte should be 0, but has value %02x\n", trailingNullByte);
