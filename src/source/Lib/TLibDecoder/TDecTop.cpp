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

/** \file     TDecTop.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "TDecTop.h"
#include "TDecConformance.h"

//! \ingroup TLibDecoder
//! \{

TDecTop::TDecTop()
  : m_iMaxRefPicNum(0)
  , m_associatedIRAPType(NAL_UNIT_INVALID)
  , m_pocCRA(0)
  , m_pocRandomAccess(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(NULL)
  , m_SEIs()
  , m_cPrediction()
  , m_cTrQuant()
  , m_cGopDecoder()
  , m_cSliceDecoder()
  , m_cCuDecoder()
  , m_cEntropyDecoder()
  , m_cCavlcDecoder()
  , m_cSbacDecoder()
  , m_cBinCABAC()
  , m_seiReader()
  , m_cLoopFilter()
  , m_cSAO()
  , m_pcPic(NULL)
  , m_prevPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_bFirstSliceInPicture(true)
#if JVET_X0048_X0103_FILM_GRAIN
  , m_bFirstPictureInSequence(true)
  , m_grainCharacteristic()
  , m_grainBuf()
#endif
  , m_bFirstSliceInSequence(true)
  , m_prevSliceSkipped(false)
  , m_skippedPOC(0)
  , m_bFirstSliceInBitstream(true)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_craNoRaslOutputFlag(false)
#if O0043_BEST_EFFORT_DECODING
  , m_forceDecodeBitDepth(8)
#endif
  , m_pDecodedSEIOutputStream(NULL)
  , m_warningMessageSkipPicture(false)
#if MCTS_ENC_CHECK
  , m_tmctsCheckEnabled(false)
#endif
  , m_prefixSEINALUs()
{
#if ENC_DEC_TRACE
  if (g_hTrace == NULL)
  {
    g_hTrace = fopen( "TraceDec.txt", "wb" );
  }
  g_bJustDoIt = g_bEncDecTraceDisable;
  g_nSymbolCounter = 0;
#endif
}

TDecTop::~TDecTop()
{
#if ENC_DEC_TRACE
  if (g_hTrace != stdout)
  {
    fclose( g_hTrace );
  }
#endif
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

Void TDecTop::create()
{
  m_cGopDecoder.create();
  m_apcSlicePilot = new TComSlice;
  m_uiSliceIdx = 0;
}

Void TDecTop::destroy()
{
  m_cGopDecoder.destroy();

  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

  m_cSliceDecoder.destroy();
}

Void TDecTop::init()
{
  // initialize ROM
  initROM();
  m_cGopDecoder.init( &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
  m_cSliceDecoder.init( &m_cEntropyDecoder, &m_cCuDecoder, &m_conformanceCheck );
#if MCTS_ENC_CHECK
  m_cEntropyDecoder.init(&m_cPrediction, &m_conformanceCheck );
#else
  m_cEntropyDecoder.init(&m_cPrediction);
#endif
}

Void TDecTop::deletePicBuffer ( )
{
  TComList<TComPic*>::iterator  iterPic   = m_cListPic.begin();
  Int iSize = Int( m_cListPic.size() );

  for (Int i = 0; i < iSize; i++ )
  {
    TComPic* pcPic = *(iterPic++);
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
  }

  m_cSAO.destroy();

  m_cLoopFilter.        destroy();

  // destroy ROM
  destroyROM();
}

Void TDecTop::xGetNewPicBuffer ( const TComSPS &sps, const TComPPS &pps, TComPic*& rpcPic, const UInt temporalLayer )
{
  m_iMaxRefPicNum = sps.getMaxDecPicBuffering(temporalLayer);     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  if (m_cListPic.size() < (UInt)m_iMaxRefPicNum)
  {
    rpcPic = new TComPic();

#if REDUCED_ENCODER_MEMORY
    rpcPic->create ( sps, pps, false, true
#if SHUTTER_INTERVAL_SEI_PROCESSING
                    , getShutterFilterFlag()
#endif
#if JVET_X0048_X0103_FILM_GRAIN
                    , false
#endif
                    );
#else
    rpcPic->create ( sps, pps, true
#if SHUTTER_INTERVAL_SEI_PROCESSING
                    , getShutterFilterFlag()
#endif
#if JVET_X0048_X0103_FILM_GRAIN
                    , false
#endif
                    );
#endif

    m_cListPic.pushBack( rpcPic );

    return;
  }

  Bool bBufferIsAvailable = false;
  TComList<TComPic*>::iterator  iterPic   = m_cListPic.begin();
  while (iterPic != m_cListPic.end())
  {
    rpcPic = *(iterPic++);
    if ( rpcPic->getReconMark() == false && rpcPic->getOutputMark() == false)
    {
      rpcPic->setOutputMark(false);
      bBufferIsAvailable = true;
      break;
    }

    if ( rpcPic->getSlice( 0 )->isReferenced() == false  && rpcPic->getOutputMark() == false)
    {
      rpcPic->setOutputMark(false);
      rpcPic->setReconMark( false );
      rpcPic->getPicYuvRec()->setBorderExtension( false );
      bBufferIsAvailable = true;
      break;
    }
  }

  if ( !bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;
    rpcPic = new TComPic();
    m_cListPic.pushBack( rpcPic );
  }
  rpcPic->destroy();
#if REDUCED_ENCODER_MEMORY
  rpcPic->create ( sps, pps, false, true
#if SHUTTER_INTERVAL_SEI_PROCESSING
                  , getShutterFilterFlag()
#endif
#if JVET_X0048_X0103_FILM_GRAIN
                  , false
#endif
                  );
#else
  rpcPic->create ( sps, pps, true
#if SHUTTER_INTERVAL_SEI_PROCESSING
                  , getShutterFilterFlag()
#endif
#if JVET_X0048_X0103_FILM_GRAIN
                  , false
#endif
                  );
#endif
}

Void TDecTop::executeLoopFilters(Int& poc, TComList<TComPic*>*& rpcListPic)
{
  if (!m_pcPic)
  {
    /* nothing to deblock */
    return;
  }

  TComPic*   pcPic         = m_pcPic;

  // Execute Deblock + Cleanup

  m_cGopDecoder.filterPicture(pcPic);

  TComSlice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcPic->getSlice(m_uiSliceIdx-1)->getPOC();
  rpcListPic          = &m_cListPic;
  m_cCuDecoder.destroy();
  m_bFirstSliceInPicture  = true;

  return;
}

Void TDecTop::checkNoOutputPriorPics (TComList<TComPic*>* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics)
  {
    return;
  }

  TComList<TComPic*>::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    TComPic* pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->setOutputMark(false);
    }
  }
}

Void TDecTop::xCreateLostPicture(Int iLostPoc)
{
  printf("\ninserting lost poc : %d\n",iLostPoc);
  TComPic *cFillPic;
  xGetNewPicBuffer(*(m_parameterSetManager.getFirstSPS()), *(m_parameterSetManager.getFirstPPS()), cFillPic, 0);
  cFillPic->getSlice(0)->initSlice();

