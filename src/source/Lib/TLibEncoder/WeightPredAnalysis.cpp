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

/** \file     WeightPredAnalysis.cpp
    \brief    weighted prediction encoder class
*/

#include "../TLibCommon/CommonDef.h"
#include "../TLibCommon/TComSlice.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComPicYuv.h"
#include "WeightPredAnalysis.h"
#include <limits>

static const Double WEIGHT_PRED_SAD_RELATIVE_TO_NON_WEIGHT_PRED_SAD=0.99; // NOTE: U0040 used 0.95

//! calculate SAD values for both WP version and non-WP version.
static
Int64 xCalcSADvalueWP(const Int   bitDepth,
                      const Pel  *pOrgPel,
                      const Pel  *pRefPel,
                      const Int   width,
                      const Int   height,
                      const Int   orgStride,
                      const Int   refStride,
                      const Int   log2Denom,
                      const Int   weight,
                      const Int   offset,
                      const Bool  useHighPrecision);

//! calculate SAD values for both WP version and non-WP version.
static
Int64 xCalcSADvalueWPOptionalClip(const Int   bitDepth,
                                  const Pel  *pOrgPel,
                                  const Pel  *pRefPel,
                                  const Int   width,
                                  const Int   height,
                                  const Int   orgStride,
                                  const Int   refStride,
                                  const Int   log2Denom,
                                  const Int   weight,
                                  const Int   offset,
                                  const Bool  useHighPrecision,
                                  const Bool  clipped);

// -----------------------------------------------------------------------------
// Helper functions


//! calculate Histogram for array of pixels
static
Void xCalcHistogram(const Pel  *pPel,
                    std::vector<Int> &histogram,
                    const Int   width,
                    const Int   height,
                    const Int   stride,
                    const Int   maxPel)
{
  histogram.clear();
  histogram.resize(maxPel);
  for( Int y = 0; y < height; y++ )
  {
    for( Int x = 0; x < width; x++ )
    {
      const Pel v=pPel[x];
      histogram[v<0?0:(v>=maxPel)?maxPel-1:v]++;
    }
    pPel += stride;
  }
}
