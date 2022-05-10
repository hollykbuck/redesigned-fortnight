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

/** \file     TDecConformance.h
    \brief    Decoder conformance functions (header)
*/

#ifndef __TDECCONFORMANCE__
#define __TDECCONFORMANCE__

// This can be enabled externally. Note that this is a PARTIAL CONFORMANCE CHECK - not a full check. More checks may be added later
#ifndef DECODER_PARTIAL_CONFORMANCE_CHECK
#define DECODER_PARTIAL_CONFORMANCE_CHECK                 0 ///< 0 (default) = do not check conformance. 1 = warn if conformance checks fail. 2 = error and quit if conformance checks fail. Note this is only a partial conformance check - not a full conformance check.
#endif



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/CommonDef.h"
#if DPB_ENCODER_USAGE_CHECK
#include "TLibCommon/ProfileLevelTierFeatures.h"
#endif
#include <stdio.h>
#include <iostream>
#if DECODER_PARTIAL_CONFORMANCE_CHECK == 2
#include <stdlib.h>
#endif


// Forward declarations
class TComSlice;
#if !DPB_ENCODER_USAGE_CHECK
class TComSPS;
class TComPPS;
#endif
class InputNALUnit;
class TComPTL;
class TComPic;

#if !DPB_ENCODER_USAGE_CHECK
typedef enum TRISTATE
{
  DISABLED=0,
  OPTIONAL=1,
  ENABLED=2
} TRISTATE;


typedef enum HBRFACTOREQN
{
  HBR_1 = 0,
  HBR_1_OR_2 = 1,
  HBR_12_OR_24 = 2
} HBRFACTOREQN;


struct LevelTierFeatures
{
  Level::Name level;
  UInt        maxLumaPs;
  UInt        maxCpb[Level::NUMBER_OF_TIERS];    // in units of CpbVclFactor or CpbNalFactor bits
  UInt        maxSliceSegmentsPerPicture;
  UInt        maxTileRows;
  UInt        maxTileCols;
  UInt64      maxLumaSr;
  UInt        maxBr[Level::NUMBER_OF_TIERS];     // in units of BrVclFactor or BrNalFactor bits/s
  UInt        minCrBase[Level::NUMBER_OF_TIERS];
