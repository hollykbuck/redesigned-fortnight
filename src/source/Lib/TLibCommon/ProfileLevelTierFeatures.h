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

/** \file     ProfileLevelTierFeatures.h
    \brief    Common profile tier level functions (header)
*/

#ifndef __PROFILELEVELTIERFEATURES__
#define __PROFILELEVELTIERFEATURES__



#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/CommonDef.h"
#include <stdio.h>
#include <iostream>


// Forward declarations
class TComSPS;
class ProfileTierLevel;


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
  UInt        getMaxPicWidthInLumaSamples()  const;
  UInt        getMaxPicHeightInLumaSamples() const;
};


struct ProfileFeatures
{

  typedef enum TRISTATE
  {
    DISABLED=0,
    OPTIONAL=1,
    ENABLED=2
  } TRISTATE;

  Profile::Name            profile;
  const TChar            *pNameString;
  UInt                     maxBitDepth;
  ChromaFormat             maxChromaFormat;
  Bool                     generalIntraConstraintFlag;
  Bool                     generalOnePictureOnlyConstraintFlag;
  TRISTATE                 generalLowerBitRateConstraint;
  TRISTATE                 generalRExtToolsEnabled;
  TRISTATE                 extendedPrecisionProcessingFlag;
  TRISTATE                 chromaQpOffsetListEnabledFlag;
