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

/** \file     TComWeightPrediction.h
    \brief    weighting prediction class (header)
*/

// Include files
#include "CommonDef.h"
#include "TComYuv.h"
#include "TComPic.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"


static inline Pel weightBidir( Int w0, Pel P0, Int w1, Pel P1, Int round, Int shift, Int offset, Int clipBD)
{
  return ClipBD( ( (w0*(P0 + IF_INTERNAL_OFFS) + w1*(P1 + IF_INTERNAL_OFFS) + round + (offset << (shift-1))) >> shift ), clipBD );
}


static inline Pel weightUnidir( Int w0, Pel P0, Int round, Int shift, Int offset, Int clipBD)
{
  return ClipBD( ( (w0*(P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clipBD );
}

static inline Pel noWeightUnidir( Pel P0, Int round, Int shift, Int offset, Int clipBD)
{
  return ClipBD( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ) + offset, clipBD );
}

static inline Pel noWeightOffsetUnidir( Pel P0, Int round, Int shift, Int clipBD)
{
  return ClipBD( ( ((P0 + IF_INTERNAL_OFFS) + round) >> shift ), clipBD );
}


// ====================================================================================================================
// Class definition
// ====================================================================================================================

TComWeightPrediction::TComWeightPrediction()
{
}


//! weighted averaging for bi-pred
Void TComWeightPrediction::addWeightBi( const TComYuv              *pcYuvSrc0,
                                        const TComYuv              *pcYuvSrc1,
                                        const BitDepths            &bitDepths,
                                        const UInt                  iPartUnitIdx,
                                        const UInt                  uiWidth,
                                        const UInt                  uiHeight,
                                        const WPScalingParam *const wp0,
                                        const WPScalingParam *const wp1,
                                              TComYuv        *const rpcYuvDst,
                                        const Bool                  bRoundLuma)
{

  const Bool enableRounding[MAX_NUM_COMPONENT]={ bRoundLuma, true, true };

  const UInt numValidComponent = pcYuvSrc0->getNumberValidComponents();

  for(Int componentIndex=0; componentIndex<numValidComponent; componentIndex++)
  {
    const ComponentID compID=ComponentID(componentIndex);

    const Pel* pSrc0       = pcYuvSrc0->getAddr( compID,  iPartUnitIdx );
    const Pel* pSrc1       = pcYuvSrc1->getAddr( compID,  iPartUnitIdx );
          Pel* pDst        = rpcYuvDst->getAddr( compID,  iPartUnitIdx );
