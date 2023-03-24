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

static
Distortion xCalcHistDistortion (const std::vector<Int> &histogram0,
                                const std::vector<Int> &histogram1)
{
  Distortion distortion = 0;
  assert(histogram0.size()==histogram1.size());
  const Int numElements=Int(histogram0.size());

  // Scan histograms to compute histogram distortion
  for (Int i = 0; i <= numElements; i++)
  {
    distortion += (Distortion)(abs(histogram0[i] - histogram1[i]));
  }

  return distortion;
}

static
void xScaleHistogram(const std::vector<Int> &histogramInput,
                           std::vector<Int> &histogramOutput, // cannot be the same as the input
                     const Int               bitDepth,
                     const Int               log2Denom,
                     const Int               weight,
                     const Int               offset,
                     const Bool              bHighPrecision)
{
  assert(&histogramInput != &histogramOutput);
  const Int numElements=Int(histogramInput.size());
  histogramOutput.clear();
  histogramOutput.resize(numElements);

  const Int64 iRealLog2Denom = bHighPrecision ? 0 : (bitDepth - 8);
  const Int64 iRealOffset    = ((Int64)offset)<<iRealLog2Denom;

  const Int divOffset = log2Denom == 0 ? 0 : 1 << (log2Denom - 1);
  // Scan histogram and apply illumination parameters appropriately
  // Then compute updated histogram.
  // Note that this technique only works with single list weights/offsets.

  for (Int i = 0; i < numElements; i++)
  {
    const Int j = Clip3(0, numElements - 1, (Int)(((weight * i + divOffset) >> log2Denom) + iRealOffset));
    histogramOutput[j] += histogramInput[i];
  }
}

static
Distortion xSearchHistogram(const std::vector<Int> &histogramSource,
                            const std::vector<Int> &histogramRef,
                                  std::vector<Int> &outputHistogram,
                            const Int               bitDepth,
                            const Int               log2Denom,
                                  Int              &weightToUpdate,
                                  Int              &offsetToUpdate,
                            const Bool              bHighPrecision,
                            const ComponentID       compID)
{
  const Int initialWeight   = weightToUpdate;
  const Int initialOffset   = offsetToUpdate;
  const Int weightRange     = 10;
  const Int offsetRange     = 10;
  const Int maxOffset       = 1 << ((bHighPrecision == true) ? (bitDepth - 1) : 7);
  const Int range           = bHighPrecision ? (1<<bitDepth) / 2 : 128;
  const Int defaultWeight   = (1<<log2Denom);
  const Int minSearchWeight = std::max<Int>(initialWeight - weightRange, defaultWeight - range);
  const Int maxSearchWeight = std::min<Int>(initialWeight + weightRange+1, defaultWeight + range);

  Distortion minDistortion   = std::numeric_limits<Distortion>::max();
  Int        bestWeight      = initialWeight;
  Int        bestOffset      = initialOffset;

  for (Int searchWeight = minSearchWeight; searchWeight < maxSearchWeight; searchWeight++)
  {
    if (compID == COMPONENT_Y)
    {
      for (Int searchOffset = std::max<Int>(initialOffset - offsetRange, -maxOffset);
               searchOffset <= initialOffset + offsetRange && searchOffset<=(maxOffset-1);
               searchOffset++)
      {
        xScaleHistogram(histogramRef, outputHistogram, bitDepth, log2Denom, searchWeight, searchOffset, bHighPrecision);
        const Distortion distortion = xCalcHistDistortion(histogramSource, outputHistogram);

        if (distortion < minDistortion)
        {
          minDistortion = distortion;
          bestWeight    = searchWeight;
          bestOffset    = searchOffset;
        }
      }
    }
    else
    {
      const Int pred        = ( maxOffset - ( ( maxOffset*searchWeight)>>(log2Denom) ) );

      for (Int searchOffset = initialOffset - offsetRange; searchOffset <= initialOffset + offsetRange; searchOffset++)
      {
        const Int deltaOffset   = Clip3( -4*maxOffset, 4*maxOffset-1, (searchOffset - pred) ); // signed 10bit (if !bHighPrecision)
        const Int clippedOffset = Clip3( -1*maxOffset, 1*maxOffset-1, (deltaOffset  + pred) ); // signed 8bit  (if !bHighPrecision)
        xScaleHistogram(histogramRef, outputHistogram, bitDepth, log2Denom, searchWeight, clippedOffset, bHighPrecision);
