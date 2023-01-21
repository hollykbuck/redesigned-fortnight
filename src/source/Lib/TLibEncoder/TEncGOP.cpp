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
