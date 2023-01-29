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

/** \file     TEncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>
#include <cinttypes>

#include "TEncTop.h"
#include "TEncGOP.h"
#include "TEncAnalyze.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/NAL.h"
#include "NALwrite.h"
#include <time.h>
#include <math.h>

#include <deque>
using namespace std;

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
Int getLSB(Int poc, Int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}

TEncGOP::TEncGOP()
{
  m_iLastIDR            = 0;
  m_RASPOCforResetEncoder = MAX_INT;

  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
  m_iLastRecoveryPicPOC = 0;

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;

  m_pcEntropyCoder      = NULL;
  m_pcCavlcCoder        = NULL;
  m_pcSbacCoder         = NULL;
  m_pcBinCABAC          = NULL;

  m_bSeqFirst           = true;

  m_bRefreshPending     = 0;
  m_pocCRA            = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_lastBPSEI         = 0;
  m_bufferingPeriodSEIPresentInAU = false;
  m_associatedIRAPType = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC  = 0;
  m_pcDeblockingTempPicYuv = NULL;
}

TEncGOP::~TEncGOP()
{
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
Void  TEncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}

Void  TEncGOP::destroy()
{
  if (m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv->destroy();
    delete m_pcDeblockingTempPicYuv;
    m_pcDeblockingTempPicYuv = NULL;
  }
#if JVET_X0048_X0103_FILM_GRAIN
  if (m_pcCfg->getFilmGrainAnalysisEnabled())
  {
    m_FGAnalyser.destroy();
  }
#endif
}

Void TEncGOP::init ( TEncTop* pcTEncTop )
{
  m_pcEncTop     = pcTEncTop;
  m_pcCfg                = pcTEncTop;
  m_seiEncoder.init(m_pcCfg, pcTEncTop, this);
  m_pcSliceEncoder       = pcTEncTop->getSliceEncoder();
  m_pcListPic            = pcTEncTop->getListPic();

  m_pcEntropyCoder       = pcTEncTop->getEntropyCoder();
  m_pcCavlcCoder         = pcTEncTop->getCavlcCoder();
  m_pcSbacCoder          = pcTEncTop->getSbacCoder();
  m_pcBinCABAC           = pcTEncTop->getBinCABAC();
  m_pcLoopFilter         = pcTEncTop->getLoopFilter();

  m_pcSAO                = pcTEncTop->getSAO();
  m_pcRateCtrl           = pcTEncTop->getRateCtrl();
  m_lastBPSEI          = 0;
  m_totalCoded         = 0;
#if JVET_X0048_X0103_FILM_GRAIN
  if (m_pcCfg->getFilmGrainAnalysisEnabled())
  {
      m_FGAnalyser.init(m_pcCfg->getSourceWidth(), m_pcCfg->getSourceHeight(), m_pcCfg->getSourcePadding(0),
          m_pcCfg->getSourcePadding(1), IPCOLOURSPACE_UNCHANGED, false, m_pcCfg->getChromaFormatIdc(),
          *(BitDepths*)m_pcCfg->getBitDepthInput(), *(BitDepths*)m_pcCfg->getBitDepth(),
          m_pcCfg->getFrameSkip(), m_pcCfg->getFGCSEICompModelPresent(),
          m_pcCfg->getFilmGrainExternalMask(), m_pcCfg->getFilmGrainExternalDenoised());

  }
#endif

}

#if MCTS_EXTRACTION
Void TEncGOP::generateVPS_RBSP(TComBitIf* rbsp, const TComVPS *vps)
{
  m_pcEntropyCoder->setBitstream(rbsp);
  m_pcEntropyCoder->encodeVPS(vps);
}

Void TEncGOP::generateSPS_RBSP(TComBitIf* rbsp, const TComSPS *sps)
{
  m_pcEntropyCoder->setBitstream(rbsp);
  m_pcEntropyCoder->encodeSPS(sps);
}

Void TEncGOP::generatePPS_RBSP(TComBitIf* rbsp, const TComPPS *pps)
{
  m_pcEntropyCoder->setBitstream(rbsp);
  m_pcEntropyCoder->encodePPS(pps);
}
#endif

Int TEncGOP::xWriteVPS (AccessUnit &accessUnit, const TComVPS *vps)
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
  m_pcEntropyCoder->encodeVPS(vps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

Int TEncGOP::xWriteSPS (AccessUnit &accessUnit, const TComSPS *sps)
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
  m_pcEntropyCoder->encodeSPS(sps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;

}

Int TEncGOP::xWritePPS (AccessUnit &accessUnit, const TComPPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
  m_pcEntropyCoder->encodePPS(pps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}


Int TEncGOP::xWriteParameterSets (AccessUnit &accessUnit, TComSlice *slice, const Bool bSeqFirst)
{
  Int actualTotalBits = 0;

  if (bSeqFirst)
  {
    actualTotalBits += xWriteVPS(accessUnit, m_pcEncTop->getVPS());
  }
  if (m_pcEncTop->SPSNeedsWriting(slice->getSPS()->getSPSId())) // Note this assumes that all changes to the SPS are made at the TEncTop level prior to picture creation (TEncTop::xGetNewPicBuffer).
  {
    assert(bSeqFirst); // Implementations that use more than 1 SPS need to be aware of activation issues.
    actualTotalBits += xWriteSPS(accessUnit, slice->getSPS());
  }
  if (m_pcEncTop->PPSNeedsWriting(slice->getPPS()->getPPSId())) // Note this assumes that all changes to the PPS are made at the TEncTop level prior to picture creation (TEncTop::xGetNewPicBuffer).
  {
    actualTotalBits += xWritePPS(accessUnit, slice->getPPS());
  }

  return actualTotalBits;
}

Void TEncGOP::xWriteAccessUnitDelimiter (AccessUnit &accessUnit, TComSlice *slice)
{
  AUDWriter audWriter;
  OutputNALUnit nalu(NAL_UNIT_ACCESS_UNIT_DELIMITER);

  Int picType = slice->isIntra() ? 0 : (slice->isInterP() ? 1 : 2);

  audWriter.codeAUD(nalu.m_Bitstream, picType);
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
Void TEncGOP::xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu(naluType, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, sps, false);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

Void TEncGOP::xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, temporalId);
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, sps, false);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

Void TEncGOP::xClearSEIs(SEIMessages& seiMessages, Bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
Void TEncGOP::xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, Bool testWrite)
{
  AccessUnit::iterator itNalu = accessUnit.begin();

  while ( (itNalu!=accessUnit.end())&&
    ( (*itNalu)->m_nalUnitType==NAL_UNIT_ACCESS_UNIT_DELIMITER 
    || (*itNalu)->m_nalUnitType==NAL_UNIT_VPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_SPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_PPS
    ))
  {
    itNalu++;
  }

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;
  
#if ENC_DEC_TRACE
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)
#if JCTVC_AD0021_SEI_MANIFEST
  // When SEI Manifest SEI message is present in an SEI NAL unit, the SEI Manifest SEI message shall be the first SEI message in the SEI NAL unit (D3.45 in ISO/IEC 23008-2).
  if (m_pcCfg->getSEIManifestSEIEnabled())
  {
    currentMessages = extractSeisByType(localMessages, SEI::SEI_MANIFEST);
    assert(currentMessages.size() <= 1);
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
    xClearSEIs(currentMessages, !testWrite);
  }
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  // When SEI Manifest SEI message is present in an SEI NAL unit, the SEI Manifest SEI message shall be the first SEI message in the SEI NAL unit (D3.45 in ISO/IEC 23008-2).
  if (m_pcCfg->getSEIPrefixIndicationSEIEnabled())
  {
    currentMessages = extractSeisByType(localMessages, SEI::SEI_PREFIX_INDICATION);
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
    xClearSEIs(currentMessages, !testWrite);
  }
#endif

  // Active parameter sets SEI must always be the first SEI
  currentMessages = extractSeisByType(localMessages, SEI::ACTIVE_PARAMETER_SETS);
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);
  
  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::BUFFERING_PERIOD);
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  currentMessages = extractSeisByType(localMessages, SEI::PICTURE_TIMING);
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
    xClearSEIs(currentMessages, !testWrite);
  }

  // Scalable nesting SEI must always be the following DU info
  currentMessages = extractSeisByType(localMessages, SEI::SCALABLE_NESTING);
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // And finally everything else one by one
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }
}


Void TEncGOP::xWriteLeadingSEIMessages (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData)
{
  AccessUnit testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PICTURE_TIMING);
  assert (picTimingSEIs.size() < 2);
  SEIPictureTiming * picTiming = picTimingSEIs.empty() ? NULL : (SEIPictureTiming*) picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, sps, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming);
  // actual writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, sps, false);

  // testAU will automatically be cleaned up when losing scope
}

Void TEncGOP::xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps)
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId, sps);
  deleteSEIs(seiMessages);
}

Void TEncGOP::xWriteDuSEIMessages (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData)
{
  const TComHRD *hrd = sps->getVuiParameters()->getHrdParameters();

  if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int naluIdx = 0;
    AccessUnit::iterator nalu = accessUnit.begin();

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu!=accessUnit.end())
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (Int duIdx = 1; duIdx < duData.size(); duIdx++)
    {
      if (duSEI == duInfoSeiMessages.end())
      {
        // if the number of generated SEIs matches the number of DUs, this should not happen
        assert (false);
        return;
      }
      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId, sps);
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu!=accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }
      duSEI++;
    }
  }
  deleteSEIs(duInfoSeiMessages);
}

#if MCTS_EXTRACTION
Void TEncGOP::xCreateIRAPLeadingSEIMessages(SEIMessages& seiMessages, const TComVPS *vps, const TComSPS *sps, const TComPPS *pps)
#else
Void TEncGOP::xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const TComSPS *sps, const TComPPS *pps)
#endif
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled())
  {
    SEIActiveParameterSets *sei = new SEIActiveParameterSets;
    m_seiEncoder.initSEIActiveParameterSets (sei, m_pcCfg->getVPS(), sps);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking (sei, m_iNumPicCoded);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getSegmentedRectFramePackingArrangementSEIEnabled())
  {
    SEISegmentedRectFramePacking *sei = new SEISegmentedRectFramePacking;
    m_seiEncoder.initSEISegmentedRectFramePacking(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = new SEIDisplayOrientation;
    m_seiEncoder.initSEIDisplayOrientation(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = new SEIToneMappingInfo;
    m_seiEncoder.initSEIToneMappingInfo (sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getTMCTSSEIEnabled())
  {
    SEITempMotionConstrainedTileSets *sei = new SEITempMotionConstrainedTileSets;
    m_seiEncoder.initSEITempMotionConstrainedTileSets(sei, pps);
    seiMessages.push_back(sei);
  }
#if MCTS_EXTRACTION
  if (m_pcCfg->getTMCTSExtractionSEIEnabled())
  {
    SEIMCTSExtractionInfoSet *sei = new SEIMCTSExtractionInfoSet;
    m_seiEncoder.initSEIMCTSExtractionInfo(sei, vps, sps, pps);
    seiMessages.push_back(sei);
  }
#endif
  if(m_pcCfg->getTimeCodeSEIEnabled())
  {
    SEITimeCode *seiTimeCode = new SEITimeCode;
    m_seiEncoder.initSEITimeCode(seiTimeCode);
    seiMessages.push_back(seiTimeCode);
  }

  if(m_pcCfg->getKneeSEIEnabled())
  {
    SEIKneeFunctionInfo *sei = new SEIKneeFunctionInfo;
    m_seiEncoder.initSEIKneeFunctionInfo(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getCcvSEIEnabled())
  {
    SEIContentColourVolume *seiContentColourVolume = new SEIContentColourVolume;
    m_seiEncoder.initSEIContentColourVolume(seiContentColourVolume);
    seiMessages.push_back(seiContentColourVolume);
  }

#if SHUTTER_INTERVAL_SEI_MESSAGE
  if (m_pcCfg->getSiiSEIEnabled())
  {
    SEIShutterIntervalInfo *seiShutterInterval = new SEIShutterIntervalInfo;
    m_seiEncoder.initSEIShutterIntervalInfo(seiShutterInterval);
    seiMessages.push_back(seiShutterInterval);
  }
#endif

#if SEI_ENCODER_CONTROL
#if JVET_X0048_X0103_FILM_GRAIN
  if (m_pcCfg->getFilmGrainCharactersticsSEIEnabled() && !m_pcCfg->getFilmGrainCharactersticsSEIPerPictureSEI())
  {
    SEIFilmGrainCharacteristics* seiFGC = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(seiFGC);
    if (m_pcCfg->getFilmGrainAnalysisEnabled())
    {
      seiFGC->m_log2ScaleFactor = m_FGAnalyser.getLog2scaleFactor();
      for (int compIdx = 0; compIdx < getNumberValidComponents(m_pcCfg->getChromaFormatIdc()); compIdx++)
      {
        if (seiFGC->m_compModel[compIdx].bPresentFlag)
        {   // higher importance of presentFlag is from cfg file
          seiFGC->m_compModel[compIdx] = m_FGAnalyser.getCompModel(compIdx);
        }
      }
    }
    seiMessages.push_back(seiFGC);
}
#else
  // film grain
  if (m_pcCfg->getFilmGrainCharactersticsSEIEnabled())
  {
    SEIFilmGrainCharacteristics *seiFGC = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(seiFGC);
    seiMessages.push_back(seiFGC);
  }
#endif
// content light level
  if (m_pcCfg->getCLLSEIEnabled())
  {
    SEIContentLightLevelInfo *seiCLL = new SEIContentLightLevelInfo;
    m_seiEncoder.initSEIContentLightLevel(seiCLL);
    seiMessages.push_back(seiCLL);
  }
// ambient viewing environment
  if (m_pcCfg->getAmbientViewingEnvironmentSEIEnabled())
  {
    SEIAmbientViewingEnvironment *seiAVE = new SEIAmbientViewingEnvironment;
    m_seiEncoder.initSEIAmbientViewingEnvironment(seiAVE);
    seiMessages.push_back(seiAVE);
  }
#endif
  if (m_pcCfg->getErpSEIEnabled())
  {
    SEIEquirectangularProjection *sei = new SEIEquirectangularProjection;
    m_seiEncoder.initSEIErp(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getSphereRotationSEIEnabled())
  {
    SEISphereRotation *sei = new SEISphereRotation;
    m_seiEncoder.initSEISphereRotation(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getOmniViewportSEIEnabled())
  {
    SEIOmniViewport *sei = new SEIOmniViewport;
    m_seiEncoder.initSEIOmniViewport(sei);
    seiMessages.push_back(sei);
  }
  if (m_pcCfg->getCmpSEIEnabled())
  {
    SEICubemapProjection *seiCubemapProjection = new SEICubemapProjection;
    m_seiEncoder.initSEICubemapProjection(seiCubemapProjection);
    seiMessages.push_back(seiCubemapProjection);
  }
  if (m_pcCfg->getRwpSEIEnabled())
  {
    SEIRegionWisePacking *seiRegionWisePacking = new SEIRegionWisePacking;
    m_seiEncoder.initSEIRegionWisePacking(seiRegionWisePacking);
    seiMessages.push_back(seiRegionWisePacking);
  }
  if (m_pcCfg->getFviSEIEnabled())
  {
    SEIFisheyeVideoInfo *sei = new SEIFisheyeVideoInfo;
    m_seiEncoder.initSEIFisheyeVideoInfo(sei);
    seiMessages.push_back(sei);
  }
    
  if(m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    const TComSEIMasteringDisplay &seiCfg=m_pcCfg->getMasteringDisplaySEI();
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    sei->values = seiCfg;
    seiMessages.push_back(sei);
  }
  if(m_pcCfg->getChromaResamplingFilterHintEnabled())
  {
    SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint = new SEIChromaResamplingFilterHint;
    m_seiEncoder.initSEIChromaResamplingFilterHint(seiChromaResamplingFilterHint, m_pcCfg->getChromaResamplingHorFilterIdc(), m_pcCfg->getChromaResamplingVerFilterIdc());
    seiMessages.push_back(seiChromaResamplingFilterHint);
  }
  if(m_pcCfg->getSEIAlternativeTransferCharacteristicsSEIEnable())
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics = new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics(seiAlternativeTransferCharacteristics);
    seiMessages.push_back(seiAlternativeTransferCharacteristics);
  }
}

Void TEncGOP::xCreatePerPictureSEIMessages (Int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, TComSlice *slice)
{
  if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    ( ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
    || ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI, slice);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_pcCfg->getScalableNestingSEIEnabled())
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }
  }

  if (picInGOP ==0 && m_pcCfg->getSOPDescriptionSEIEnabled() ) // write SOP description SEI (if enabled) at the beginning of GOP
  {
    SEISOPDescription* sopDescriptionSEI = new SEISOPDescription();
    m_seiEncoder.initSEISOPDescription(sopDescriptionSEI, slice, picInGOP, m_iLastIDR, m_iGopSize);
    seiMessages.push_back(sopDescriptionSEI);
  }

  if( ( m_pcEncTop->getRecoveryPointSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) )
  {
    if( m_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !slice->getRapPicFlag() )
    {
      // Gradual decoding refresh SEI
      SEIRegionRefreshInfo *gradualDecodingRefreshInfoSEI = new SEIRegionRefreshInfo();
      gradualDecodingRefreshInfoSEI->m_gdrForegroundFlag = true; // Indicating all "foreground"
      seiMessages.push_back(gradualDecodingRefreshInfoSEI);
    }
    // Recovery point SEI
    SEIRecoveryPoint *recoveryPointSEI = new SEIRecoveryPoint();
    m_seiEncoder.initSEIRecoveryPoint(recoveryPointSEI, slice);
    seiMessages.push_back(recoveryPointSEI);
  }
  if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
  {
    SEITemporalLevel0Index *temporalLevel0IndexSEI = new SEITemporalLevel0Index();
    m_seiEncoder.initTemporalLevel0IndexSEI(temporalLevel0IndexSEI, slice);
    seiMessages.push_back(temporalLevel0IndexSEI);
  }

  if( m_pcEncTop->getNoDisplaySEITLayer() && ( slice->getTLayer() >= m_pcEncTop->getNoDisplaySEITLayer() ) )
  {
    SEINoDisplay *seiNoDisplay = new SEINoDisplay;
    seiNoDisplay->m_noDisplay = true;
    seiMessages.push_back(seiNoDisplay);
  }

  // insert one Colour Remapping Info SEI for the picture (if the file exists)
  if (!m_pcCfg->getColourRemapInfoSEIFileRoot().empty())
  {
    SEIColourRemappingInfo *seiColourRemappingInfo = new SEIColourRemappingInfo();
    const Bool success = m_seiEncoder.initSEIColourRemappingInfo(seiColourRemappingInfo, slice->getPOC() );

    if(success)
    {
      seiMessages.push_back(seiColourRemappingInfo);
    }
    else
    {
      delete seiColourRemappingInfo;
    }
  }

  // insert one Annotated Region SEI for the picture (if the file exists)
  if (!m_pcCfg->getAnnotatedRegionSEIFileRoot().empty())
  {
    SEIAnnotatedRegions *seiAnnotatedRegions = new SEIAnnotatedRegions();
    const Bool success = m_seiEncoder.initSEIAnnotatedRegions(seiAnnotatedRegions, slice->getPOC());

    if (success)
    {
      seiMessages.push_back(seiAnnotatedRegions);
    }
    else
    {
      delete seiAnnotatedRegions;
    }
  }
  // insert one Regional Nesting SEI for the picture (if the file exists)
  if (!m_pcCfg->getRegionalNestingSEIFileRoot().empty())
  {
    SEIRegionalNesting *seiRegionalNesting= new SEIRegionalNesting();
    const Bool success = m_seiEncoder.initSEIRegionalNesting(seiRegionalNesting, slice->getPOC() );

    if(success)
    {
      seiMessages.push_back(seiRegionalNesting);
    }
    else
    {
      delete seiRegionalNesting;
    }
  }
#if JVET_X0048_X0103_FILM_GRAIN
  // Film Grain Characteristics SEI insertion at at frame level
  if (m_pcCfg->getFilmGrainCharactersticsSEIEnabled() && m_pcCfg->getFilmGrainCharactersticsSEIPerPictureSEI())
  {
    SEIFilmGrainCharacteristics* fgcSEI = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(fgcSEI);
    if (m_pcCfg->getFilmGrainAnalysisEnabled())
    {
      fgcSEI->m_log2ScaleFactor = m_FGAnalyser.getLog2scaleFactor();
      for (int compIdx = 0; compIdx < getNumberValidComponents(m_pcCfg->getChromaFormatIdc()); compIdx++)
      {
        if (fgcSEI->m_compModel[compIdx].bPresentFlag)
        {   // higher importance of presentFlag is from cfg file
          fgcSEI->m_compModel[compIdx] = m_FGAnalyser.getCompModel(compIdx);
        }
      }
    }
    seiMessages.push_back(fgcSEI);
  }
#endif

#if JCTVC_AD0021_SEI_MANIFEST
  // Make sure that sei_manifest and sei_prefix are the last two initialized sei_msg, otherwise it will cause these two
  // Sei messages to not be able to enter all SEI messages
  if (m_pcCfg->getSEIManifestSEIEnabled())
  {
    SEIManifest* seiSEIManifest = new SEIManifest;
    m_seiEncoder.initSEISEIManifest(seiSEIManifest, seiMessages);
    seiMessages.push_back(seiSEIManifest);
  }
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (m_pcCfg->getSEIPrefixIndicationSEIEnabled())
  {
    int NumOfSEIPrefixMsg = 0;
    for (auto& it : seiMessages)
    {
      if (it->payloadType() == SEI::SEI_MANIFEST)
      {
        break;
      }
      NumOfSEIPrefixMsg++;
    }
    for (auto& it : seiMessages)
    {
      if (NumOfSEIPrefixMsg == 0 || it->payloadType() == SEI::SEI_MANIFEST)
      {
        break;
      }
      SEIPrefixIndication* seiSEIPrefixIndication = new SEIPrefixIndication;
      m_seiEncoder.initSEISEIPrefixIndication(seiSEIPrefixIndication, it);
      seiMessages.push_back(seiSEIPrefixIndication);
      NumOfSEIPrefixMsg--;
    }
  }
#endif
}

Void TEncGOP::xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages)
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI* sei=nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}

Void TEncGOP::xCreatePictureTimingSEI  (Int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, TComSlice *slice, Bool isField, std::deque<DUData> &duData)
{
  const TComVUI *vui = slice->getSPS()->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();

  // update decoding unit parameters
  if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    (  hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() ) )
  {
    Int picSptDpbOutputDuDelay = 0;
    SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

    // DU parameters
    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      UInt numDU = (UInt) duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1     = ( numDU - 1 );
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize( numDU );
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize( numDU );
    }
    pictureTimingSEI->m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<Double>(hrd->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
    pictureTimingSEI->m_picDpbOutputDelay = slice->getSPS()->getNumReorderPics(slice->getSPS()->getMaxTLayers()-1) + slice->getPOC() - m_totalCoded;
    if(m_pcCfg->getEfficientFieldIRAPEnabled() && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay ++;
    }
    Int factor = hrd->getTickDivisorMinus2() + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
    {
      picSptDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    }
    if (m_bufferingPeriodSEIPresentInAU)
    {
      m_lastBPSEI = m_totalCoded;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      Int i;
      UInt64 ui64Tmp;
      UInt uiPrev = 0;
      UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
      std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
      UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

      for( i = 0; i < numDU; i ++ )
      {
        pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
      }

      if( numDU == 1 )
      {
        rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
      }
      else
      {
        rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
        UInt tmp = 0;
        UInt accum = 0;

        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
          if( (UInt)ui64Tmp > maxDiff )
          {
            tmp ++;
          }
        }
        uiPrev = 0;

        UInt flag = 0;
        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          flag = 0;
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

          if( (UInt)ui64Tmp > maxDiff )
          {
            if(uiPrev >= maxDiff - tmp)
            {
              ui64Tmp = uiPrev + 1;
              flag = 1;
            }
            else                            ui64Tmp = maxDiff - tmp + 1;
          }
          rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
          if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
          {
            rDuCpbRemovalDelayMinus1[ i ] = 0;
          }
          else if (tmp > 0 && flag == 1)
          {
            tmp --;
          }
          accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
          uiPrev = accum;
        }
      }
    }
    
    if( m_pcCfg->getPictureTimingSEIEnabled() )
    {
      pictureTimingSEI->m_picStruct = (isField && slice->getPic()->isTopField())? 1 : isField? 2 : 0;
      seiMessages.push_back(pictureTimingSEI);

      if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
    {
      for( Int i = 0; i < ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx = i;
        duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
        duInfoSEI->m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if( !m_pcCfg->getPictureTimingSEIEnabled() && pictureTimingSEI )
    {
      delete pictureTimingSEI;
    }
  }
}

Void TEncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first 
  UInt numNalUnits = (UInt)testAU.size();
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += ( numRBSPBytes << 3 );
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (Int i=1; i<duData.size(); i++)
  {
    if (m_pcCfg->getDecodingUnitInfoSEIEnabled())
    {
      numNalUnits  += 1;
      numRBSPBytes += ( 5 << 3 );
    }
    duData[i].accumBitsDU += numRBSPBytes; // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
  {
    duData.back().accumBitsDU += ( 20 << 3 ); // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }

}
Void TEncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const TComSPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();
  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int i;
    UInt64 ui64Tmp;
    UInt uiPrev = 0;
    UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
    std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

    for( i = 0; i < numDU; i ++ )
    {
      pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
    }

    if( numDU == 1 )
    {
      rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
      UInt tmp = 0;
      UInt accum = 0;

      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
        if( (UInt)ui64Tmp > maxDiff )
        {
          tmp ++;
        }
      }
      uiPrev = 0;

      UInt flag = 0;
      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        flag = 0;
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

        if( (UInt)ui64Tmp > maxDiff )
        {
          if(uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag = 1;
          }
          else                            ui64Tmp = maxDiff - tmp + 1;
        }
        rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
        if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
        {
          rDuCpbRemovalDelayMinus1[ i ] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp --;
        }
        accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
        uiPrev = accum;
      }
    }
  }
}
Void TEncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == NULL))
  {
    return;
  }

  Int i=0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du!= duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo*) (*du);
    duInfoSEI->m_decodingUnitIdx = i;
    duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static Void
cabac_zero_word_padding(TComSlice *const pcSlice, TComPic *const pcPic, const std::size_t binCountsInNalUnits, const std::size_t numBytesInVclNalUnits, std::ostringstream &nalUnitData, const Bool cabacZeroWordPaddingEnabled)
{
  const TComSPS &sps=*(pcSlice->getSPS());
  const Int log2subWidthCxsubHeightC = (pcPic->getComponentScaleX(COMPONENT_Cb)+pcPic->getComponentScaleY(COMPONENT_Cb));
  const Int minCuWidth  = pcPic->getMinCUWidth();
  const Int minCuHeight = pcPic->getMinCUHeight();
  const Int paddedWidth = ((sps.getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;
  const Int paddedHeight= ((sps.getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;
  const Int rawBits = paddedWidth * paddedHeight *
                         (sps.getBitDepth(CHANNEL_TYPE_LUMA) + ((2*sps.getBitDepth(CHANNEL_TYPE_CHROMA))>>log2subWidthCxsubHeightC));
  const std::size_t threshold = (32/3)*numBytesInVclNalUnits + (rawBits/32);
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+31)/32;

    if (targetNumBytesInVclNalUnits>numBytesInVclNalUnits) // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded=targetNumBytesInVclNalUnits - numBytesInVclNalUnits;
      const std::size_t numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
      const std::size_t numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<UChar> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, UChar(0));
        for(std::size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i*3+2]=3;  // 00 00 03
        }
        nalUnitData.write(reinterpret_cast<const TChar*>(&(zeroBytesPadding[0])), numberOfAdditionalCabacZeroBytes);
        printf("Adding %d bytes of padding\n", UInt(numberOfAdditionalCabacZeroWords*3));
      }
      else
      {
        printf("Standard would normally require adding %d bytes of padding\n", UInt(numberOfAdditionalCabacZeroWords*3));
      }
    }
  }
}

class EfficientFieldIRAPMapping
{
  private:
    Int  IRAPGOPid;
    Bool IRAPtoReorder;
    Bool swapIRAPForward;

  public:
    EfficientFieldIRAPMapping() :
      IRAPGOPid(-1),
      IRAPtoReorder(false),
      swapIRAPForward(false)
    { }

    Void initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, TEncGOP *pEncGop, TEncCfg *pCfg);

    Int adjustGOPid(const Int gopID);
    Int restoreGOPid(const Int gopID);
    Int GetIRAPGOPid() const { return IRAPGOPid; }
};

Void EfficientFieldIRAPMapping::initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, TEncGOP *pEncGop, TEncCfg *pCfg )
{
  if(isField)
  {
    Int pocCurr;
    for ( Int iGOPid=0; iGOPid < gopSize; iGOPid++ )
    {
      // determine actual POC
      if(POCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(POCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + pCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < gopSize-1 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true; 
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false; 
          break;
        }
      }
    }
  }
}

Int EfficientFieldIRAPMapping::adjustGOPid(const Int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return IRAPGOPid;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid -1)
      {
        return IRAPGOPid;
      }
      else if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
    }
  }
  return GOPid;
}

Int EfficientFieldIRAPMapping::restoreGOPid(const Int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        IRAPtoReorder = false;
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return GOPid -1;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
      else if(GOPid == IRAPGOPid -1)
      {
        IRAPtoReorder = false;
        return IRAPGOPid;
      }
    }
  }
  return GOPid;
}


static UInt calculateCollocatedFromL0Flag(const TComSlice *pSlice)
{
  const Int refIdx = 0; // Zero always assumed
  const TComPic *refPicL0 = pSlice->getRefPic(REF_PIC_LIST_0, refIdx);
  const TComPic *refPicL1 = pSlice->getRefPic(REF_PIC_LIST_1, refIdx);
  return refPicL0->getSlice(0)->getSliceQp() > refPicL1->getSlice(0)->getSliceQp();
}


static Void
printHash(const HashType hashType, const std::string &digestStr)
{
  const TChar *decodedPictureHashModeName;
  switch (hashType)
  {
    case HASHTYPE_MD5:
      decodedPictureHashModeName = "MD5";
      break;
    case HASHTYPE_CRC:
      decodedPictureHashModeName = "CRC";
      break;
    case HASHTYPE_CHECKSUM:
      decodedPictureHashModeName = "Checksum";
      break;
    default:
      decodedPictureHashModeName = NULL;
      break;
  }
  if (decodedPictureHashModeName != NULL)
  {
    if (digestStr.empty())
    {
      printf(" [%s:%s]", decodedPictureHashModeName, "?");
    }
    else
    {
      printf(" [%s:%s]", decodedPictureHashModeName, digestStr.c_str());
    }
  }
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void TEncGOP::compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic,
                           TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP,
                           Bool isField, Bool isTff, const InputColourSpaceConversion ip_conversion, const InputColourSpaceConversion snr_conversion, const TEncAnalyze::OutputLogControl &outputLogCtrl )
{
  // TODO: Split this function up.

  TComPic*        pcPic = NULL;
  TComPicYuv*     pcPicYuvRecOut;
  TComSlice*      pcSlice;
  TComOutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new TComOutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted

  xInitGOP( iPOCLast, iNumPicRcvd, isField );

  m_iNumPicCoded = 0;
  SEIMessages leadingSeiMessages;
  SEIMessages nestedSeiMessages;
  SEIMessages duInfoSeiMessages;
  SEIMessages trailingSeiMessages;
  std::deque<DUData> duData;
  SEIDecodingUnitInfo decodingUnitInfoSEI;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, iPOCLast, iNumPicRcvd, m_iLastIDR, this, m_pcCfg);
  }

  // reset flag indicating whether pictures have been encoded
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    m_pcCfg->setEncodedFlag(iGOPid, false);
  }

  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.adjustGOPid(iGOPid);
    }

    //-- For time output for each slice
    clock_t iBeforeTime = clock();


    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    Int iTimeOffset;
    Int pocCurr;

    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = 1;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1:0);
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
    {
      if (m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
      }
      continue;
    }

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }
    // start a new access unit: create an entry in the list of output access units
    accessUnitsInGOP.push_back(AccessUnit());
    AccessUnit& accessUnit = accessUnitsInGOP.back();
    xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isField );

#if REDUCED_ENCODER_MEMORY
#if SHUTTER_INTERVAL_SEI_PROCESSING
    pcPic->prepareForReconstruction( m_pcCfg->getShutterFilterFlag() );
#else
    pcPic->prepareForReconstruction();
#endif

#endif
    //  Slice data initialization
    pcPic->clearSliceBuffer();
    pcPic->allocateNewSlice();
    m_pcSliceEncoder->setSliceIdx(0);
    pcPic->setCurrSliceIdx(0);

    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iGOPid, pcSlice, isField );

    pcSlice->setLastIDR(m_iLastIDR);
    pcSlice->setSliceIdx(0);
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }
    
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
    if(pcSlice->getTemporalLayerNonReferenceFlag())
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }
    // Do decoding refresh marking if any
    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcCfg->getEfficientFieldIRAPEnabled());
    m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
    if (!m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }

    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP()) 
      || (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
      )
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3, m_pcCfg->getEfficientFieldIRAPEnabled());
    }

    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());

    if(pcSlice->getTLayer() > 0 
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
      {
        if(pcSlice->getTemporalLayerNonReferenceFlag())
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
      }
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        Bool isSTSA=true;
        for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer())
          {
            const TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                Int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                  {
                    break;
                  }
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA=false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)
        {
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }
      }
    }
    arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);
    pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));

    //  Set reference list
    pcSlice->setRefPicList ( rcListPic );

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }


    if (pcSlice->getPOC() > m_RASPOCforResetEncoder && m_pcCfg->getResetEncoderStateAfterIRAP())
    {
      // need to reset encoder decisions.
      m_pcSliceEncoder->resetEncoderDecisions();

      if (pcSlice->getSPS()->getUseSAO())
      {
        m_pcSAO->resetEncoderDecisions();
      }
      m_RASPOCforResetEncoder=MAX_INT;
    }
    if (pcSlice->isIRAP())
    {
      m_RASPOCforResetEncoder = pcSlice->getPOC();
    }

    pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
#if MCTS_EXTRACTION
    SliceType  encCABACTableIdx = pcSlice->getEncCABACTableIdx();
    Bool encCabacInitFlag = (pcSlice->getSliceType() != encCABACTableIdx && encCABACTableIdx != I_SLICE) ? true : false;
    pcSlice->setCabacInitFlag(encCabacInitFlag);
#endif

    if (pcSlice->getSliceType() == B_SLICE)
    {
      const UInt uiColFromL0 = calculateCollocatedFromL0Flag(pcSlice);
      pcSlice->setColFromL0Flag(uiColFromL0);
      Bool bLowDelay = true;
      Int  iCurrPOC  = pcSlice->getPOC();
      Int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }


    //-------------------------------------------------------------
    pcSlice->setRefPOCList();

    pcSlice->setList1IdxToList0Idx();

    if (m_pcEncTop->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        pcSlice->setEnableTMVPFlag(1);
      }
    }
    else if (m_pcEncTop->getTMVPModeId() == 1)
    {
      pcSlice->setEnableTMVPFlag(1);
    }
    else
    {
      pcSlice->setEnableTMVPFlag(0);
    }
    
    // set adaptive search range for non-intra-slices
    if (m_pcCfg->getUseASR() && pcSlice->getSliceType()!=I_SLICE)
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    Bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        Int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      pcSlice->setMvdL1ZeroFlag(true);
    }
    else
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }


    Double lambda            = 0.0;
    Int actualHeadBits       = 0;
    Int actualTotalBits      = 0;
    Int estimatedBits        = 0;
    Int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() ) // TODO: does this work with multiple slices and slice-segments?
    {
      Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->getSlice(0)->getSliceType() == I_SLICE )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

      if (m_pcRateCtrl->getCpbSaturationEnabled() && frameLevel != 0)
      {
        Int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

        // prevent overflow
        if (estimatedCpbFullness - estimatedBits > (Int)(m_pcRateCtrl->getCpbSize()*0.9f))
        {
          estimatedBits = estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.9f);
        }

        estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
        // prevent underflow
        if (estimatedCpbFullness - estimatedBits < m_pcRateCtrl->getRCPic()->getLowerBound())
        {
          estimatedBits = max(200, estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound());
        }

        m_pcRateCtrl->getRCPic()->setTargetBits(estimatedBits);
      }

      Int sliceQP = m_pcCfg->getInitialQP();
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
      {
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
        Double dQPFactor     = 0.57*dLambda_scale;
        Int    SHIFT_QP      = 12;
        Int    bitdepth_luma_qp_scale = 0;
        Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic); // TODO: This only analyses the first slice segment - what about the others?

        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );

          if (m_pcRateCtrl->getCpbSaturationEnabled() )
          {
            Int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

            // prevent overflow
            if (estimatedCpbFullness - bits > (Int)(m_pcRateCtrl->getCpbSize()*0.9f))
            {
              bits = estimatedCpbFullness - (Int)(m_pcRateCtrl->getCpbSize()*0.9f);
            }

            estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
            // prevent underflow
            if (estimatedCpbFullness - bits < m_pcRateCtrl->getRCPic()->getLowerBound())
            {
              bits = estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound();
            }
          }

          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }

        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }

      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }

    UInt uiNumSliceSegments = 1;

    // Allocate some coders, now the number of tiles are known.
    const Int numSubstreamsColumns = (pcSlice->getPPS()->getNumTileColumnsMinus1() + 1);
    const Int numSubstreamRows     = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcPic->getFrameHeightInCtus() : (pcSlice->getPPS()->getNumTileRowsMinus1() + 1);
    const Int numSubstreams        = numSubstreamRows * numSubstreamsColumns;
    std::vector<TComOutputBitstream> substreamsOut(numSubstreams);

    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      const UInt numberOfCtusInFrame=pcPic->getPicSym()->getNumberOfCtusInFrame();
      pcSlice->setSliceCurStartCtuTsAddr( 0 );
      pcSlice->setSliceSegmentCurStartCtuTsAddr( 0 );

      for(UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
      {
        m_pcSliceEncoder->precompressSlice( pcPic );
        m_pcSliceEncoder->compressSlice   ( pcPic, false, false );

        const UInt curSliceSegmentEnd = pcSlice->getSliceSegmentCurEndCtuTsAddr();
        if (curSliceSegmentEnd < numberOfCtusInFrame)
        {
          const Bool bNextSegmentIsDependentSlice=curSliceSegmentEnd<pcSlice->getSliceCurEndCtuTsAddr();
          const UInt sliceBits=pcSlice->getSliceBits();
          pcPic->allocateNewSlice();
          // prepare for next slice
          pcPic->setCurrSliceIdx                    ( uiNumSliceSegments );
          m_pcSliceEncoder->setSliceIdx             ( uiNumSliceSegments   );
          pcSlice = pcPic->getSlice                 ( uiNumSliceSegments   );
          assert(pcSlice->getPPS()!=0);
          pcSlice->copySliceInfo                    ( pcPic->getSlice(uiNumSliceSegments-1)  );
          pcSlice->setSliceIdx                      ( uiNumSliceSegments   );
          if (bNextSegmentIsDependentSlice)
          {
            pcSlice->setSliceBits(sliceBits);
          }
          else
          {
            pcSlice->setSliceCurStartCtuTsAddr      ( curSliceSegmentEnd );
            pcSlice->setSliceBits(0);
          }
          pcSlice->setDependentSliceSegmentFlag(bNextSegmentIsDependentSlice);
          pcSlice->setSliceSegmentCurStartCtuTsAddr ( curSliceSegmentEnd );
          // TODO: optimise cabac_init during compress slice to improve multi-slice operation
          // pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
          uiNumSliceSegments ++;
        }
        nextCtuTsAddr = curSliceSegmentEnd;
      }
    }

    duData.clear();
    pcSlice = pcPic->getSlice(0);

    // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
    if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary() )
    {
      m_pcSAO->getPreDBFStatistics(pcPic);
    }

    //-- Loop filter
    Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
    m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
      if ( m_pcCfg->getDeblockingFilterMetric()==2 )
      {
        applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
      }
      else
      {
        applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
      }
    }
    m_pcLoopFilter->loopFilterPic( pcPic );

#if JVET_X0048_X0103_FILM_GRAIN
    if (m_pcCfg->getFilmGrainAnalysisEnabled())
    {
      int  filteredFrame = m_pcCfg->getIntraPeriod() < 1 ? 2 * m_pcCfg->getFrameRate() : m_pcCfg->getIntraPeriod();
      bool ready_to_analyze = pcPic->getPOC() % filteredFrame ? false : true; // either it is mctf denoising or external source for film grain analysis. note: if mctf is used, it is different from mctf for encoding.
      if (ready_to_analyze)
      {
          m_FGAnalyser.initBufs(pcPic);
          m_FGAnalyser.estimate_grain(pcPic);
      }
    }
#endif

    /////////////////////////////////////////////////////////////////////////////////////////////////// File writing
    // Set entropy coder
    m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );

    // write various parameter sets
    //bool writePS = m_bSeqFirst || (m_pcCfg->getReWriteParamSetsFlag() && (pcPic->getSlice(0)->getSliceType() == I_SLICE));
    bool writePS = m_bSeqFirst || (m_pcCfg->getReWriteParamSetsFlag() && (pcSlice->isIRAP()));
    if (writePS)
    {
      m_pcEncTop->setParamSetChanged(pcSlice->getSPS()->getSPSId(), pcSlice->getPPS()->getPPSId());
    }
    actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, writePS);

    if (writePS)
    {
      // create prefix SEI messages at the beginning of the sequence
      assert(leadingSeiMessages.empty());
#if MCTS_EXTRACTION
      xCreateIRAPLeadingSEIMessages(leadingSeiMessages, m_pcEncTop->getVPS(),  pcSlice->getSPS(), pcSlice->getPPS());
#else
      xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->getSPS(), pcSlice->getPPS());
#endif

      m_bSeqFirst = false;
    }
    if (m_pcCfg->getAccessUnitDelimiter())
    {
      xWriteAccessUnitDelimiter(accessUnit, pcSlice);
    }

    // reset presence of BP SEI indication
    m_bufferingPeriodSEIPresentInAU = false;
    // create prefix SEI associated with a picture
    xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

    /* use the main bitstream buffer for storing the marshalled picture */
    m_pcEntropyCoder->setBitstream(NULL);

    pcSlice = pcPic->getSlice(0);

    if (pcSlice->getSPS()->getUseSAO())
    {
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      TComBitCounter tempBitCounter;
      tempBitCounter.resetBits();
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(&tempBitCounter);
      m_pcSAO->initRDOCabacCoder(m_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
      m_pcSAO->SAOProcess(pcPic, sliceEnabled, pcPic->getSlice(0)->getLambdas(),
                          m_pcCfg->getTestSAODisableAtPictureLevel(),
                          m_pcCfg->getSaoEncodingRate(),
                          m_pcCfg->getSaoEncodingRateChroma(),
                          m_pcCfg->getSaoCtuBoundary());
      m_pcSAO->PCMLFDisableProcess(pcPic);
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(NULL);

      //assign SAO slice header
      for(Int s=0; s< uiNumSliceSegments; s++)
      {
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
        assert(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]);
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
      }
    }

    // pcSlice is currently slice 0.
    std::size_t binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
    std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

    for( UInt sliceSegmentStartCtuTsAddr = 0, sliceIdxCount=0; sliceSegmentStartCtuTsAddr < pcPic->getPicSym()->getNumberOfCtusInFrame(); sliceIdxCount++, sliceSegmentStartCtuTsAddr=pcSlice->getSliceSegmentCurEndCtuTsAddr() )
    {
      pcSlice = pcPic->getSlice(sliceIdxCount);
      if(sliceIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)
      {
        pcSlice->checkColRefIdx(sliceIdxCount, pcPic);
      }
      pcPic->setCurrSliceIdx(sliceIdxCount);
      m_pcSliceEncoder->setSliceIdx(sliceIdxCount);

      pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
      pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());

      for ( UInt ui = 0 ; ui < numSubstreams; ui++ )
      {
        substreamsOut[ui].clear();
      }

      m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );
      m_pcEntropyCoder->resetEntropy      ( pcSlice );
      /* start slice NALunit */
      OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

      pcSlice->setNoRaslOutputFlag(false);
      if (pcSlice->isIRAP())
      {
        if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
        {
          pcSlice->setNoRaslOutputFlag(true);
        }
        //the inference for NoOutputPriorPicsFlag
        // KJS: This cannot happen at the encoder
        if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
        {
          if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
          {
            pcSlice->setNoOutputPriorPicsFlag(true);
          }
        }
      }

      pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
#if MCTS_EXTRACTION
      encCABACTableIdx = pcSlice->getEncCABACTableIdx();
      encCabacInitFlag = (pcSlice->getSliceType() != encCABACTableIdx && encCABACTableIdx != I_SLICE) ? true : false;
      pcSlice->setCabacInitFlag(encCabacInitFlag);
#endif
      tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();
      m_pcEntropyCoder->encodeSliceHeader(pcSlice);
      actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );

      pcSlice->setFinalized(true);

      pcSlice->clearSubstreamSizes(  );
      {
        UInt numBinsCoded = 0;
        m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
        binCountsInNalUnits+=numBinsCoded;
      }

      {
        // Construct the final bitstream by concatenating substreams.
        // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
        // Complete the slice header info.
        m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
        m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );

        // Append substreams...
        TComOutputBitstream *pcOut = pcBitstreamRedirect;
        const Int numZeroSubstreamsAtStartOfSlice  = pcPic->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
        const Int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
        for ( UInt ui = 0 ; ui < numSubstreamsToCode; ui++ )
        {
          pcOut->addSubstream(&(substreamsOut[ui+numZeroSubstreamsAtStartOfSlice]));
        }
      }

      // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
      // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
      Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
