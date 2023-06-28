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
        assert(trailingNullByte==0);
      }
    }
  }
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnaryMaxSymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, UInt uiMaxSymbol )
#endif
{
  if (uiMaxSymbol == 0)
  {
    ruiSymbol = 0;
    return;
  }

  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );

  if( ruiSymbol == 0 || uiMaxSymbol == 1 )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol++;
  } while( uiCont && ( uiSymbol < uiMaxSymbol - 1 ) );

  if( uiCont && ( uiSymbol == uiMaxSymbol - 1 ) )
  {
    uiSymbol++;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
#endif
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;

  while( uiBit )
  {
    m_pcTDecBinIf->decodeBinEP( uiBit RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += uiBit << uiCount++;
  }

  if ( --uiCount )
  {
    UInt bins;
    m_pcTDecBinIf->decodeBinsEP( bins, uiCount RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    uiSymbol += bins;
  }

  ruiSymbol = uiSymbol;
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadUnarySymbol( UInt& ruiSymbol, ContextModel* pcSCModel, Int iOffset )
#endif
{
  m_pcTDecBinIf->decodeBin( ruiSymbol, pcSCModel[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));

  if( !ruiSymbol )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    m_pcTDecBinIf->decodeBin( uiCont, pcSCModel[ iOffset ] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    uiSymbol++;
  } while( uiCont );

  ruiSymbol = uiSymbol;
}


/** Parsing of coeff_abs_level_remaing
 * \param rSymbol                 reference to coeff_abs_level_remaing
 * \param rParam                  reference to parameter
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 */
#if RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange, const class TComCodingStatisticsClassType &whichStat )
#else
Void TDecSbac::xReadCoefRemainExGolomb ( UInt &rSymbol, UInt &rParam, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange )
#endif
{
  UInt prefix   = 0;
  UInt codeWord = 0;

  if (useLimitedPrefixLength)
  {
    const UInt longestPossiblePrefix = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange)) + COEF_REMAIN_BIN_REDUCTION;

    do
    {
      prefix++;
      m_pcTDecBinIf->decodeBinEP( codeWord RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    } while((codeWord != 0) && (prefix < longestPossiblePrefix));
  }
  else
  {
    do
    {
      prefix++;
      m_pcTDecBinIf->decodeBinEP( codeWord RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat) );
    } while( codeWord);
  }

  codeWord  = 1 - codeWord;
  prefix -= codeWord;
  codeWord=0;

  if (prefix < COEF_REMAIN_BIN_REDUCTION )
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (prefix<<rParam) + codeWord;
  }
  else if (useLimitedPrefixLength)
  {
    const UInt maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

    const UInt prefixLength = prefix - COEF_REMAIN_BIN_REDUCTION;
    const UInt suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - rParam) : prefixLength;

    m_pcTDecBinIf->decodeBinsEP(codeWord, (suffixLength + rParam) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));

    rSymbol = codeWord + ((((1 << prefixLength) - 1) + COEF_REMAIN_BIN_REDUCTION) << rParam);
  }
  else
  {
    m_pcTDecBinIf->decodeBinsEP(codeWord,prefix-COEF_REMAIN_BIN_REDUCTION+rParam RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(whichStat));
    rSymbol = (((1<<(prefix-COEF_REMAIN_BIN_REDUCTION))+COEF_REMAIN_BIN_REDUCTION-1)<<rParam)+codeWord;
  }
}


/** Parse I_PCM information.
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 *
 * If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
 */
Void TDecSbac::parseIPCMInfo ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  m_pcTDecBinIf->decodeBinTrm(uiSymbol);

  if (uiSymbol == 1)
  {
    Bool bIpcmFlag = true;
    const TComSPS &sps=*(pcCU->getSlice()->getSPS());

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( sps.getMaxCUWidth()>>uiDepth, sps.getMaxCUHeight()>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setTrIdxSubParts     ( 0, uiAbsPartIdx, uiDepth );
    pcCU->setIPCMFlagSubParts  ( bIpcmFlag, uiAbsPartIdx, uiDepth );

    const UInt minCoeffSizeY = pcCU->getPic()->getMinCUWidth() * pcCU->getPic()->getMinCUHeight();
    const UInt offsetY       = minCoeffSizeY * uiAbsPartIdx;
    for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
    {
      const ComponentID compID = ComponentID(ch);
      const UInt offset = offsetY >> (pcCU->getPic()->getComponentScaleX(compID) + pcCU->getPic()->getComponentScaleY(compID));
      Pel * pPCMSample  = pcCU->getPCMSample(compID) + offset;
      const UInt width  = pcCU->getWidth (uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleX(compID);
      const UInt height = pcCU->getHeight(uiAbsPartIdx) >> pcCU->getPic()->getComponentScaleY(compID);
      const UInt sampleBits = pcCU->getSlice()->getSPS()->getPCMBitDepth(toChannelType(compID));
      for (UInt y=0; y<height; y++)
      {
        for (UInt x=0; x<width; x++)
        {
          UInt sample;
          m_pcTDecBinIf->xReadPCMCode(sampleBits, sample);
          pPCMSample[x] = sample;
        }
        pPCMSample += width;
      }
    }

    m_pcTDecBinIf->start();
  }
}

Void TDecSbac::parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_CUTransquantBypassFlagSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__TQ_BYPASS_FLAG) );
  pcCU->setCUTransquantBypassSubParts(uiSymbol ? true : false, uiAbsPartIdx, uiDepth);
}

/** parse skip flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCtxSkip = pcCU->getCtxSkipFlag( uiAbsPartIdx );
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSkipFlagSCModel.get( 0, 0, uiCtxSkip ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__SKIP_FLAG) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tSkipFlag" );
  DTRACE_CABAC_T( "\tuiCtxSkip: ");
  DTRACE_CABAC_V( uiCtxSkip );
  DTRACE_CABAC_T( "\tuiSymbol: ");
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\n");

  if( uiSymbol )
  {
    pcCU->setSkipFlagSubParts( true,        uiAbsPartIdx, uiDepth );
    pcCU->setPredModeSubParts( MODE_INTER,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth, pcCU->getSlice()->getSPS()->getMaxCUHeight()>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setMergeFlagSubParts( true , uiAbsPartIdx, 0, uiDepth );
  }
}


/** parse merge flag
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \param uiPUIdx
 * \returns Void
 */
Void TDecSbac::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, *m_cCUMergeFlagExtSCModel.get( 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_FLAG) );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );

  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tMergeFlag: " );
  DTRACE_CABAC_V( uiSymbol );
  DTRACE_CABAC_T( "\tAddress: " );
  DTRACE_CABAC_V( pcCU->getCtuRsAddr() );
  DTRACE_CABAC_T( "\tuiAbsPartIdx: " );
  DTRACE_CABAC_V( uiAbsPartIdx );
  DTRACE_CABAC_T( "\n" );
}

Void TDecSbac::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex )
{
  UInt uiUnaryIdx = 0;
  UInt uiNumCand = pcCU->getSlice()->getMaxNumMergeCand();
  if ( uiNumCand > 1 )
  {
    for( ; uiUnaryIdx < uiNumCand - 1; ++uiUnaryIdx )
    {
      UInt uiSymbol = 0;
      if ( uiUnaryIdx==0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUMergeIdxExtSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MERGE_INDEX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
  }
  ruiMergeIndex = uiUnaryIdx;

  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tparseMergeIndex()" )
  DTRACE_CABAC_T( "\tuiMRGIdx= " )
  DTRACE_CABAC_V( ruiMergeIndex )
  DTRACE_CABAC_T( "\n" )
}

Void TDecSbac::parseMVPIdx      ( Int& riMVPIdx )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, m_cMVPIdxSCModel.get(0), 1, AMVP_MAX_NUM_CANDS-1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVP_IDX) );
  riMVPIdx = uiSymbol;
}

Void TDecSbac::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__SPLIT_FLAG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth]+2);
#endif

  UInt uiSymbol;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUSplitFlagSCModel.get( 0, 0, pcCU->getCtxSplitFlag( uiAbsPartIdx, uiDepth ) ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  DTRACE_CABAC_VL( g_nSymbolCounter++ )
  DTRACE_CABAC_T( "\tSplitFlag\n" )
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return;
}

/** parse partition size
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol, uiMode = 0;
  PartSize eMode;
  const UChar cuWidth =UChar(pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth);
  const UChar cuHeight=UChar(pcCU->getSlice()->getSPS()->getMaxCUHeight()>>uiDepth);
  const Int log2DiffMaxMinCodingBlockSize = pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize();

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__PART_SIZE, g_aucConvertToBit[cuWidth]+2);
#endif

  assert ( pcCU->getSlice()->getSPS()->getLog2DiffMaxMinCodingBlockSize() == log2DiffMaxMinCodingBlockSize);
  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    uiSymbol = 1;
    if( uiDepth == log2DiffMaxMinCodingBlockSize )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    }
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
    UInt uiTrLevel = 0;
    UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
    UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
    uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;
    if( eMode == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
  else
  {
    UInt uiMaxNumBits = 2;

    if( uiDepth == log2DiffMaxMinCodingBlockSize && !( cuWidth == 8 && cuHeight == 8 ) )
    {
      uiMaxNumBits ++;
    }

    for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, ui) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if ( uiSymbol )
      {
        break;
      }
      uiMode++;
    }
    eMode = (PartSize) uiMode;
    if ( pcCU->getSlice()->getSPS()->getUseAMP() && uiDepth < log2DiffMaxMinCodingBlockSize )
    {
      if (eMode == SIZE_2NxN)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype));
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        m_pcTDecBinIf->decodeBin(uiSymbol, m_cCUPartSizeSCModel.get( 0, 0, 3 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        if (uiSymbol == 0)
        {
          m_pcTDecBinIf->decodeBinEP(uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
  }
  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( cuWidth, cuHeight, uiAbsPartIdx, uiDepth );
}


/** parse prediction mode
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TDecSbac::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;
  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUPredModeSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__PRED_MODE) );
  iPredMode += uiSymbol;
  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt absPartIdx, UInt depth )
{
  PartSize mode = pcCU->getPartitionSize( absPartIdx );
  UInt partNum = mode==SIZE_NxN?4:1;
  UInt partOffset = ( pcCU->getPic()->getNumPartitionsInCtu() >> ( pcCU->getDepth(absPartIdx) << 1 ) ) >> 2;
  UInt mpmPred[4],symbol;
  Int j,intraPredMode;
  if (mode==SIZE_NxN)
  {
    depth++;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>depth]+2, CHANNEL_TYPE_LUMA);
#endif
  for (j=0;j<partNum;j++)
  {
    m_pcTDecBinIf->decodeBin( symbol, m_cCUIntraPredSCModel.get( 0, 0, 0) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    mpmPred[j] = symbol;
  }
  for (j=0;j<partNum;j++)
  {
    Int preds[NUM_MOST_PROBABLE_MODES] = {-1, -1, -1};
    pcCU->getIntraDirPredictor(absPartIdx+partOffset*j, preds, COMPONENT_Y);
    if (mpmPred[j])
    {
      m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      if (symbol)
      {
        m_pcTDecBinIf->decodeBinEP( symbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        symbol++;
      }
      intraPredMode = preds[symbol];
    }
    else
    {
      m_pcTDecBinIf->decodeBinsEP( symbol, 5 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
      intraPredMode = symbol;

      //postponed sorting of MPMs (only in remaining branch)
      if (preds[0] > preds[1])
      {
        std::swap(preds[0], preds[1]);
      }
      if (preds[0] > preds[2])
      {
        std::swap(preds[0], preds[2]);
      }
      if (preds[1] > preds[2])
      {
        std::swap(preds[1], preds[2]);
      }
      for ( UInt i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        intraPredMode += ( intraPredMode >= preds[i] );
      }
    }
    pcCU->setIntraDirSubParts(CHANNEL_TYPE_LUMA, (UChar)intraPredMode, absPartIdx+partOffset*j, depth );
  }
}


Void TDecSbac::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__INTRA_DIR_ANG, g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxCUWidth()>>uiDepth]+2, CHANNEL_TYPE_CHROMA);
#endif

  m_pcTDecBinIf->decodeBin( uiSymbol, m_cCUChromaPredSCModel.get( 0, 0, 0 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
  if( uiSymbol == 0 )
  {
    uiSymbol = DM_CHROMA_IDX;
  }
  else
  {
    UInt uiIPredMode;
    m_pcTDecBinIf->decodeBinsEP( uiIPredMode, 2 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
    UInt uiAllowedChromaDir[ NUM_CHROMA_MODE ];
    pcCU->getAllowedChromaDir( uiAbsPartIdx, uiAllowedChromaDir );
    uiSymbol = uiAllowedChromaDir[ uiIPredMode ];
  }

  pcCU->setIntraDirSubParts( CHANNEL_TYPE_CHROMA, uiSymbol, uiAbsPartIdx, uiDepth );
}


Void TDecSbac::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx )
{
  UInt uiSymbol;
  const UInt uiCtx = pcCU->getCtxInterDir( uiAbsPartIdx );
  ContextModel *pCtx = m_cCUInterDirSCModel.get( 0 );

  uiSymbol = 0;
  if (pcCU->getPartitionSize(uiAbsPartIdx) == SIZE_2Nx2N || pcCU->getHeight(uiAbsPartIdx) != 8 )
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + uiCtx ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
  }

  if( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiSymbol, *( pCtx + 4 ) RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__INTER_DIR) );
    assert(uiSymbol == 0 || uiSymbol == 1);
  }

  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecSbac::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList )
{
  UInt uiSymbol;

  ContextModel *pCtx = m_cCURefPicSCModel.get( 0 );
  m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );

  if( uiSymbol )
  {
    UInt uiRefNum = pcCU->getSlice()->getNumRefIdx( eRefList ) - 2;
    pCtx++;
    UInt ui;
    for( ui = 0; ui < uiRefNum; ++ui )
    {
      if( ui == 0 )
      {
        m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      else
      {
        m_pcTDecBinIf->decodeBinEP( uiSymbol RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__REF_FRM_IDX) );
      }
      if( uiSymbol == 0 )
      {
        break;
      }
    }
    uiSymbol = ui + 1;
  }
  riRefFrmIdx = uiSymbol;

  return;
}

Void TDecSbac::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  UInt uiHorAbs;
  UInt uiVerAbs;
  UInt uiHorSign = 0;
  UInt uiVerSign = 0;
  ContextModel *pCtx = m_cCUMvdSCModel.get( 0 );

  if(pcCU->getSlice()->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pcCU->getInterDir(uiAbsPartIdx)==3)
  {
    uiHorAbs=0;
    uiVerAbs=0;
  }
  else
  {
    m_pcTDecBinIf->decodeBin( uiHorAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
    m_pcTDecBinIf->decodeBin( uiVerAbs, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );

    const Bool bHorAbsGr0 = uiHorAbs != 0;
    const Bool bVerAbsGr0 = uiVerAbs != 0;
    pCtx++;

    if( bHorAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiHorAbs += uiSymbol;
    }

    if( bVerAbsGr0 )
    {
      m_pcTDecBinIf->decodeBin( uiSymbol, *pCtx RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD) );
      uiVerAbs += uiSymbol;
    }

    if( bHorAbsGr0 )
    {
      if( 2 == uiHorAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiHorAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiHorSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

    if( bVerAbsGr0 )
    {
      if( 2 == uiVerAbs )
      {
        xReadEpExGolomb( uiSymbol, 1 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
        uiVerAbs += uiSymbol;
      }

      m_pcTDecBinIf->decodeBinEP( uiVerSign RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(STATS__CABAC_BITS__MVD_EP) );
    }

  }

  const TComMv cMv( uiHorSign ? -Int( uiHorAbs ): uiHorAbs, uiVerSign ? -Int( uiVerAbs ) : uiVerAbs );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiDepth, uiPartIdx );
  return;
}

Void TDecSbac::parseCrossComponentPrediction( TComTU &rTu, ComponentID compID )
{
  TComDataCU *pcCU = rTu.getCU();

  if( isLuma(compID) || !pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() )
  {
    return;
  }

  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();

  if (!pcCU->isIntra(uiAbsPartIdx) || (pcCU->getIntraDir( CHANNEL_TYPE_CHROMA, uiAbsPartIdx ) == DM_CHROMA_IDX))
  {
    SChar alpha = 0;
    UInt symbol = 0;

    DTRACE_CABAC_VL( g_nSymbolCounter++ )
    DTRACE_CABAC_T("\tparseCrossComponentPrediction()")
    DTRACE_CABAC_T( "\tAddr=" )
    DTRACE_CABAC_V( compID )
    DTRACE_CABAC_T( "\tuiAbsPartIdx=" )
    DTRACE_CABAC_V( uiAbsPartIdx )
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatisticsClassType ctype(STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION, (g_aucConvertToBit[rTu.getRect(compID).width] + 2), compID);
#endif
    ContextModel *pCtx = m_cCrossComponentPredictionSCModel.get(0, 0) + ((compID == COMPONENT_Cr) ? (NUM_CROSS_COMPONENT_PREDICTION_CTX >> 1) : 0);
    m_pcTDecBinIf->decodeBin( symbol, pCtx[0] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

    if(symbol != 0)
    {
      // Cross-component prediction alpha is non-zero.
      UInt sign = 0;
      m_pcTDecBinIf->decodeBin( symbol, pCtx[1] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

      if (symbol != 0)
      {
        // alpha is 2 (symbol=1), 4(symbol=2) or 8(symbol=3).
        // Read up to two more bits
        xReadUnaryMaxSymbol( symbol, (pCtx + 2), 1, 2 RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );
        symbol += 1;
      }
      m_pcTDecBinIf->decodeBin( sign, pCtx[4] RExt__DECODER_DEBUG_BIT_STATISTICS_PASS_OPT_ARG(ctype) );

      alpha = (sign != 0) ? -(1 << symbol) : (1 << symbol);
    }
    DTRACE_CABAC_T( "\tAlpha=" )
    DTRACE_CABAC_V( alpha )
    DTRACE_CABAC_T( "\n" )

    pcCU->setCrossComponentPredictionAlphaPartRange( alpha, compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts( compID ) );
  }
}
