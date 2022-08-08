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

/** \file     TDecEntropy.h
    \brief    entropy decoder class (header)
*/

#ifndef __TDECENTROPY__
#define __TDECENTROPY__

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/TComSlice.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComSampleAdaptiveOffset.h"
#include "TLibCommon/TComRectangle.h"
#include "TDecConformance.h"

class TDecSbac;
class TDecCavlc;
class ParameterSetManagerDecoder;
class TComPrediction;

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// entropy decoder pure class
class TDecEntropyIf
{
public:
  //  Virtual list for SBAC/CAVLC
  virtual Void  resetEntropy          ( TComSlice* pcSlice )     = 0;
  virtual Void  setBitstream          ( TComInputBitstream* p )  = 0;

  virtual Void  parseVPS                  ( TComVPS* pcVPS )     = 0;
  virtual Void  parseSPS                  ( TComSPS* pcSPS )     = 0;
  virtual Void  parsePPS                  ( TComPPS* pcPPS )     = 0;

  virtual Void parseSliceHeader          ( TComSlice* pcSlice, ParameterSetManager *parameterSetManager, const Int prevTid0POC)       = 0;

  virtual Void parseTerminatingBit       ( UInt& ruilsLast )                                     = 0;
  virtual Void parseRemainingBytes( Bool noTrailingBytesExpected ) = 0;

  virtual Void parseMVPIdx        ( Int& riMVPIdx ) = 0;

public:
  virtual Void parseSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void parseMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx ) = 0;
  virtual Void parseMergeIndex    ( TComDataCU* pcCU, UInt& ruiMergeIndex ) = 0;
  virtual Void parsePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void parsePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;

  virtual Void parseIntraDirLumaAng( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;
  virtual Void parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth ) = 0;

  virtual Void parseInterDir      ( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx ) = 0;
  virtual Void parseRefFrmIdx     ( TComDataCU* pcCU, Int& riRefFrmIdx, RefPicList eRefList ) = 0;
  virtual Void parseMvd           ( TComDataCU* pcCU, UInt uiAbsPartAddr, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList ) = 0;

  virtual Void parseCrossComponentPrediction ( class TComTU &rTu, ComponentID compID ) = 0;

  virtual Void parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize ) = 0;
  virtual Void parseQtCbf         ( TComTU &rTu, const ComponentID compID, const Bool lowestLevel ) = 0;
  virtual Void parseQtRootCbf     ( UInt uiAbsPartIdx, UInt& uiQtRootCbf ) = 0;
