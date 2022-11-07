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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride );

/// constrained intra prediction
Bool  isAboveLeftAvailable  ( const TComDataCU* pcCU, UInt uiPartIdxLT );
Int   isAboveAvailable      ( const TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isLeftAvailable       ( const TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
Int   isAboveRightAvailable ( const TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isBelowLeftAvailable  ( const TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );



// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

#if MCTS_ENC_CHECK
Void TComPattern::initPattern(Pel* piY,
                              Int roiWidth,
                              Int roiHeight,
                              Int stride,
                              Int bitDepthLuma,
                              Int roiPosX,
                              Int roiPosY)
#else
Void TComPattern::initPattern (Pel* piY,
                               Int roiWidth,
                               Int roiHeight,
                               Int stride,
                               Int bitDepthLuma)
#endif
{
  m_piROIOrigin = piY;
  m_roiWidth = roiWidth;
  m_roiHeight = roiHeight;
  m_patternStride = stride;
  m_bitDepth = bitDepthLuma;
#if MCTS_ENC_CHECK
  m_roiPosX       = roiPosX;
  m_roiPosY       = roiPosY;
#endif
}

#if MCTS_ENC_CHECK
Void TComPattern::setTileBorders(Int tileLeftTopPelPosX, Int tileLeftTopPelPosY, Int tileRightBottomPelPosX, Int tileRightBottomPelPosY)
{
  m_tileLeftTopPelPosX = tileLeftTopPelPosX;
  m_tileLeftTopPelPosY = tileLeftTopPelPosY;
  m_tileRightBottomPelPosX = tileRightBottomPelPosX;
  m_tileRightBottomPelPosY = tileRightBottomPelPosY;
}
#endif


// TODO: move this function to TComPrediction.cpp.
Void TComPrediction::initIntraPatternChType( TComTU &rTu, const ComponentID compID, const Bool bFilterRefSamples DEBUG_STRING_FN_DECLARE(sDebug))
{
  const ChannelType chType    = toChannelType(compID);

  TComDataCU *pcCU=rTu.getCU();
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
  const UInt uiTuWidth2       = uiTuWidth  << 1;
  const UInt uiTuHeight2      = uiTuHeight << 1;

  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;
  const Int  iLeftUnits       = iTUHeightInUnits << 1;
  const Int  bitDepthForChannel = sps.getBitDepth(chType);

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  Int   iNumIntraNeighbor = 0;

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );

  const UInt         uiROIWidth  = uiTuWidth2+1;
  const UInt         uiROIHeight = uiTuHeight2+1;

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);

#if DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piIntraTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);
#if O0043_BEST_EFFORT_DECODING
    const Int  bitDepthForChannelInStream = sps.getStreamBitDepth(chType);
    fillReferenceSamples (bitDepthForChannelInStream, bitDepthForChannelInStream - bitDepthForChannel,
#else
    fillReferenceSamples (bitDepthForChannel,
#endif
                          piRoiOrigin, piIntraTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
                          uiROIWidth, uiROIHeight, iPicStride);


#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: generating Ref Samples for channel " << compID << " and " << rTu.getRect(compID).width << " x " << rTu.getRect(compID).height << "\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << piIntraTemp[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif

    if (bFilterRefSamples)
    {
      // generate filtered intra prediction samples

            Int          stride    = uiROIWidth;
      const Pel         *piSrcPtr  = piIntraTemp                           + (stride * uiTuHeight2); // bottom left
            Pel         *piDestPtr = m_piYuvExt[compID][PRED_BUF_FILTERED] + (stride * uiTuHeight2); // bottom left

      //------------------------------------------------

      Bool useStrongIntraSmoothing = isLuma(chType) && sps.getUseStrongIntraSmoothing();

      const Pel bottomLeft = piIntraTemp[stride * uiTuHeight2];
      const Pel topLeft    = piIntraTemp[0];
      const Pel topRight   = piIntraTemp[uiTuWidth2];

      if (useStrongIntraSmoothing)
      {
#if O0043_BEST_EFFORT_DECODING
        const Int  threshold     = 1 << (bitDepthForChannelInStream - 5);
#else
        const Int  threshold     = 1 << (bitDepthForChannel - 5);
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piIntraTemp[stride * uiTuHeight])) < threshold; //difference between the
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piIntraTemp[         uiTuWidth ])) < threshold; //ends and the middle
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))
        {
          useStrongIntraSmoothing = false;
        }
      }

      *piDestPtr = *piSrcPtr; // bottom left is not filtered
      piDestPtr -= stride;
      piSrcPtr  -= stride;

      //------------------------------------------------

      //left column (bottom to top)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);
      }
      else
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      //top-left

      if (useStrongIntraSmoothing)
      {
        *piDestPtr = piSrcPtr[0];
      }
      else
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;
      piSrcPtr  += 1;

      //------------------------------------------------

      //top row (left-to-right)

      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuWidth] + 3; //log2(uiTuWidth2)

        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++)
        {
          *piDestPtr = (((uiTuWidth2 - i) * topLeft) + (i * topRight) + uiTuWidth) >> shift;
        }

        piSrcPtr += uiTuWidth2 - 1;
      }
      else
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

      *piDestPtr=*piSrcPtr; // far right is not filtered

#if DEBUG_STRING
